#include <arpa/inet.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <ctime>
#include <cstring>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <mutex>
#include <vector>

namespace {
constexpr int Port = 80;
constexpr int ReceiverCount = 4;
constexpr const char* NetplanFile = "/etc/netplan/50-cloud-init.yaml";
constexpr std::array<const char*, 3> SerialDevices{"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0"};
// The ESP32 runs as the Bluetooth SPP server; the Pi connects out to it as
// an RFCOMM client. Its MAC address must be supplied via the ESP32_BT_MAC
// environment variable (see rfid-web-cpp.service) since it can't be
// autodetected without a discovery/pairing step.
constexpr const char* Esp32BluetoothMacEnvVar = "ESP32_BT_MAC";
constexpr uint8_t Esp32RfcommChannel = 1;
constexpr const char* Esp32DeviceId = "pi-rfid";

struct AntennaState {
    bool detected = false;
    std::chrono::steady_clock::time_point lastSeen{};
};

struct Reading {
    int antenna = 0;
    std::string epc;
    int rssi = 0;
    int count = 0;
    std::string timestamp;
};

std::array<AntennaState, ReceiverCount> antennaStates;
std::mutex antennaMutex;
std::vector<Reading> readings;
std::vector<Reading> bufferReadings;
std::atomic<bool> readerRunning{true};
std::atomic<bool> readerConnected{false};
std::atomic<bool> scanEnabled{false};
std::atomic<bool> bufferMode{false};
std::atomic<bool> bufferLogAvailable{false};
std::atomic<int> requestedRfPower{-1};
std::atomic<bool> esp32BridgeRunning{true};
std::atomic<bool> esp32Connected{false};
std::mutex esp32QueueMutex;
std::deque<std::string> esp32Queue; // newline-terminated JSON lines, oldest-first

uint16_t checksum(const std::vector<unsigned char>& bytes) {
    uint16_t crc = 0xFFFF;
    for (const auto byte : bytes) {
        crc ^= byte;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408) : static_cast<uint16_t>(crc >> 1);
        }
    }
    return crc;
}

std::string jsonEscape(const std::string& value) {
    std::string out;
    out.reserve(value.size());
    for (const char ch : value) {
        switch (ch) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(ch) >= 0x20) out += ch;
        }
    }
    return out;
}

int64_t epochMillisNow() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

// See PROTOCOL.md in the esp32-display firmware project for this schema.
std::string buildStatusJson(bool scanning) {
    std::ostringstream out;
    out << "{\"type\":\"status\",\"device_id\":\"" << jsonEscape(Esp32DeviceId) << "\",\"scan_state\":" << (scanning ? 1 : 0) << "}";
    return out.str();
}

std::string buildScanResultJson(const Reading& reading) {
    std::ostringstream out;
    out << "{\"type\":\"scan_result\",\"antenna\":" << reading.antenna
        << ",\"epc\":\"" << jsonEscape(reading.epc) << "\""
        << ",\"rssi\":" << reading.rssi
        << ",\"count\":" << reading.count
        << ",\"ts\":" << epochMillisNow() << "}";
    return out.str();
}

std::string buildPingJson() {
    std::ostringstream out;
    out << "{\"type\":\"ping\",\"ts\":" << epochMillisNow() << "}";
    return out.str();
}

void queueEsp32Line(const std::string& json) {
    std::lock_guard<std::mutex> lock(esp32QueueMutex);
    if (esp32Queue.size() >= 128) esp32Queue.pop_front();
    esp32Queue.push_back(json + "\n");
}

void queueEsp32Status(bool scanning) {
    queueEsp32Line(buildStatusJson(scanning));
}

void queueEsp32Reading(const Reading& reading) {
    queueEsp32Line(buildScanResultJson(reading));
}

void queueEsp32Heartbeat() {
    queueEsp32Line(buildPingJson());
}

int connectEsp32Bluetooth(const std::string& macAddress) {
    const int sock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
    if (sock < 0) return -1;

    sockaddr_rc address{};
    address.rc_family = AF_BLUETOOTH;
    address.rc_channel = Esp32RfcommChannel;
    str2ba(macAddress.c_str(), &address.rc_bdaddr);

    if (connect(sock, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        close(sock);
        return -1;
    }
    return sock;
}

// Connects out to the ESP32 (it runs as the Bluetooth SPP server) and keeps
// sending queued JSON lines plus a periodic ping, reconnecting whenever the
// link drops.
void esp32Loop() {
    const char* macAddress = std::getenv(Esp32BluetoothMacEnvVar);
    if (macAddress == nullptr || *macAddress == '\0') {
        std::cerr << "ESP32 Bluetooth disabled: set " << Esp32BluetoothMacEnvVar
                   << " to the ESP32's Bluetooth MAC address" << std::endl;
        return;
    }

    while (esp32BridgeRunning) {
        std::cerr << "ESP32 Bluetooth: connecting to " << macAddress << " ..." << std::endl;
        const int sock = connectEsp32Bluetooth(macAddress);
        if (sock < 0) {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            continue;
        }
        std::cerr << "ESP32 Bluetooth: connected" << std::endl;
        esp32Connected = true;
        queueEsp32Status(scanEnabled);

        auto lastHeartbeat = std::chrono::steady_clock::now();
        bool linkOk = true;
        while (linkOk && esp32BridgeRunning) {
            std::string line;
            {
                std::lock_guard<std::mutex> lock(esp32QueueMutex);
                if (!esp32Queue.empty()) {
                    line = esp32Queue.front();
                    esp32Queue.pop_front();
                }
            }
            if (!line.empty()) {
                const auto written = write(sock, line.data(), line.size());
                if (written != static_cast<ssize_t>(line.size())) {
                    std::cerr << "ESP32 Bluetooth write failed; reconnecting" << std::endl;
                    linkOk = false;
                    break;
                }
            }
            const auto now = std::chrono::steady_clock::now();
            if (now - lastHeartbeat >= std::chrono::milliseconds(500)) {
                lastHeartbeat = now;
                queueEsp32Heartbeat();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        esp32Connected = false;
        close(sock);
        if (esp32BridgeRunning) std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

std::vector<unsigned char> inventoryCommand(unsigned char antenna) {
    // Reader type 0x75 follows the legacy UHFReader288M path: Q=4.
    std::vector<unsigned char> frame{13, 0xFF, 0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, antenna, 0x14};
    const auto crc = checksum(frame);
    frame.push_back(static_cast<unsigned char>(crc & 0xFF));
    frame.push_back(static_cast<unsigned char>((crc >> 8) & 0xFF));
    return frame;
}

std::vector<unsigned char> reader18InventoryCommand() {
    std::vector<unsigned char> frame{4, 0xFF, 0x01};
    const auto crc = checksum(frame);
    frame.push_back(static_cast<unsigned char>(crc & 0xFF));
    frame.push_back(static_cast<unsigned char>((crc >> 8) & 0xFF));
    return frame;
}

std::vector<unsigned char> commandFrame(unsigned char command, const std::vector<unsigned char>& data) {
    std::vector<unsigned char> frame;
    frame.push_back(static_cast<unsigned char>(4 + data.size()));
    frame.push_back(0xFF);
    frame.push_back(command);
    frame.insert(frame.end(), data.begin(), data.end());
    const auto crc = checksum(frame);
    frame.push_back(static_cast<unsigned char>(crc & 0xFF));
    frame.push_back(static_cast<unsigned char>((crc >> 8) & 0xFF));
    return frame;
}

std::string hexDump(const std::vector<unsigned char>& bytes) {
    static constexpr char hex[] = "0123456789abcdef";
    std::string result;
    for (const auto byte : bytes) {
        result.push_back(hex[byte >> 4]);
        result.push_back(hex[byte & 0x0F]);
        result.push_back(' ');
    }
    return result;
}

bool readExact(int serial, unsigned char* buffer, size_t length, int timeoutMs) {
    size_t received = 0;
    while (received < length) {
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(serial, &readSet);
        timeval timeout{timeoutMs / 1000, (timeoutMs % 1000) * 1000};
        const int ready = select(serial + 1, &readSet, nullptr, nullptr, &timeout);
        if (ready <= 0) return false;
        const ssize_t count = read(serial, buffer + received, length - received);
        if (count <= 0) return false;
        received += static_cast<size_t>(count);
    }
    return true;
}

bool readFrame(int serial, std::vector<unsigned char>& frame, int timeoutMs) {
    unsigned char length = 0;
    if (!readExact(serial, &length, 1, timeoutMs) || length < 4 || length > 250) return false;
    frame.resize(static_cast<size_t>(length) + 1);
    frame[0] = length;
    return readExact(serial, frame.data() + 1, length, timeoutMs);
}

void markDetected(int antenna) {
    if (antenna < 1 || antenna > ReceiverCount) return;
    std::lock_guard<std::mutex> lock(antennaMutex);
    antennaStates[antenna - 1].detected = true;
    antennaStates[antenna - 1].lastSeen = std::chrono::steady_clock::now();
}

std::string timestampNow() {
    const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    char buffer[32]{};
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    return buffer;
}

std::string bytesToHex(const unsigned char* bytes, size_t length) {
    static constexpr char hex[] = "0123456789ABCDEF";
    std::string result;
    result.reserve(length * 2);
    for (size_t index = 0; index < length; ++index) {
        result.push_back(hex[bytes[index] >> 4]);
        result.push_back(hex[bytes[index] & 0x0F]);
    }
    return result;
}

void parseInventoryFrame(const std::vector<unsigned char>& frame) {
    if (frame.size() < 8) return;
    if (frame[3] != 1 && frame[3] != 3) {
        std::cerr << "RFID inventory error response: " << hexDump(frame) << std::endl;
        return;
    }
    const unsigned char antennaMask = frame[4];
    const int antenna = antennaMask == 1 ? 1 : antennaMask == 2 ? 2 : antennaMask == 4 ? 3 : antennaMask == 8 ? 4 : 0;
    const int tagCount = frame[5];
    size_t cursor = 6;
    for (int tag = 0; tag < tagCount && cursor < frame.size(); ++tag) {
        const size_t tagLength = frame[cursor++];
        if (cursor + tagLength >= frame.size()) break;
        const std::string epc = bytesToHex(frame.data() + cursor, tagLength);
        cursor += tagLength;
        const int rssi = cursor < frame.size() ? frame[cursor] : 0;
        if (cursor < frame.size()) ++cursor;
        markDetected(antenna);
        std::lock_guard<std::mutex> lock(antennaMutex);
        auto& targetReadings = bufferMode ? bufferReadings : readings;
        const auto targetExisting = std::find_if(targetReadings.begin(), targetReadings.end(), [&](const Reading& reading) {
            return reading.antenna == antenna && reading.epc == epc;
        });
        if (targetExisting != targetReadings.end()) {
            ++targetExisting->count;
            targetExisting->rssi = rssi;
            targetExisting->timestamp = timestampNow();
            queueEsp32Reading(*targetExisting);
        } else {
            targetReadings.push_back({antenna, epc, rssi, 1, timestampNow()});
            queueEsp32Reading(targetReadings.back());
        }
        const size_t maxReadings = bufferMode ? 5000 : 500;
        const size_t trimCount = bufferMode ? 500 : 100;
        if (targetReadings.size() > maxReadings) targetReadings.erase(targetReadings.begin(), targetReadings.begin() + trimCount);
    }
}

void readerLoop() {
    int serial = -1;
    const char* serialDevice = nullptr;
    for (const auto device : SerialDevices) {
        serial = open(device, O_RDWR | O_NOCTTY);
        if (serial >= 0) {
            serialDevice = device;
            break;
        }
    }
    if (serial < 0) {
        std::cerr << "RFID serial unavailable: /dev/ttyUSB0, /dev/ttyUSB1, /dev/ttyACM0" << std::endl;
        return;
    }
    std::cerr << "RFID serial opened: " << serialDevice << std::endl;
    readerConnected = true;
    termios settings{};
    tcgetattr(serial, &settings);
    cfmakeraw(&settings);
    settings.c_cflag = CS8 | CREAD | CLOCAL;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 5;
    cfsetispeed(&settings, B57600);
    cfsetospeed(&settings, B57600);
    tcsetattr(serial, TCSANOW, &settings);
    tcflush(serial, TCIOFLUSH);

    const auto configure = [&](unsigned char command, const std::vector<unsigned char>& data) {
        const auto frame = commandFrame(command, data);
        std::cerr << "RFID command: " << hexDump(frame) << std::endl;
        if (write(serial, frame.data(), frame.size()) != static_cast<ssize_t>(frame.size())) return false;
        std::vector<unsigned char> response;
        if (!readFrame(serial, response, 1200)) {
            std::cerr << "RFID command timeout: " << static_cast<int>(command) << std::endl;
            return false;
        }
        std::cerr << "RFID response: " << hexDump(response) << std::endl;
        return response.size() > 3 && response[3] == 0;
    };
    // Match the Python startup order: set region/frequency and RF power first.
    // Match the legacy Python defaults: US band, steps 0..48, RF power 15.
    const bool frequencyConfigured = configure(0x22, {0x30, 0x80});
    const bool powerConfigured = configure(0x2F, {15});
    if (!frequencyConfigured || !powerConfigured) {
        std::cerr << "RFID frequency/power command was not acknowledged; continuing with reader defaults." << std::endl;
    }
    const auto infoCommand = commandFrame(0x21, {});
    std::cerr << "RFID command: " << hexDump(infoCommand) << std::endl;
    if (write(serial, infoCommand.data(), infoCommand.size()) != static_cast<ssize_t>(infoCommand.size())) {
        std::cerr << "RFID reader-info write failed" << std::endl;
        readerConnected = false;
        close(serial);
        return;
    }
    std::vector<unsigned char> infoResponse;
    if (!readFrame(serial, infoResponse, 5000)) {
        std::cerr << "RFID reader-info timeout: no response from serial device" << std::endl;
        readerConnected = false;
        close(serial);
        return;
    }
    std::cerr << "RFID reader-info response: " << hexDump(infoResponse) << std::endl;
    if (infoResponse.size() < 7 || infoResponse[3] != 0) {
        std::cerr << "RFID reader-info returned an invalid status" << std::endl;
        readerConnected = false;
        close(serial);
        return;
    }
    const unsigned char readerType = infoResponse[6];
    std::cerr << "RFID reader type: 0x" << std::hex << static_cast<int>(readerType) << std::dec << std::endl;
    const bool reader18 = readerType == 0x09;
    const bool configured = reader18 ? configure(0x35, {0, 0, 0, 0, 0, 0}) : configure(0x76, {0});
    if (!configured) {
        std::cerr << "RFID work-mode command was not acknowledged; continuing with reader default mode." << std::endl;
    }

    const std::array<unsigned char, ReceiverCount> antennas{0x80, 0x81, 0x82, 0x83};
    while (readerRunning) {
        const int requestedPower = requestedRfPower.exchange(-1);
        if (requestedPower >= 0) {
            const bool powerUpdated = configure(0x2F, {static_cast<unsigned char>(requestedPower)});
            std::cerr << "RFID runtime power update " << requestedPower << " dBm: " << (powerUpdated ? "ok" : "failed") << std::endl;
        }
        if (!scanEnabled) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        for (const auto antenna : antennas) {
            const auto command = reader18 ? reader18InventoryCommand() : inventoryCommand(antenna);
            write(serial, command.data(), command.size());
            std::vector<unsigned char> frame;
            const int responseTimeout = bufferMode ? 180 : 800;
            if (readFrame(serial, frame, responseTimeout)) parseInventoryFrame(frame);
        }
        std::lock_guard<std::mutex> lock(antennaMutex);
        const auto now = std::chrono::steady_clock::now();
        for (auto& state : antennaStates) {
            state.detected = state.lastSeen.time_since_epoch().count() != 0 && now - state.lastSeen < std::chrono::seconds(3);
        }
    }
    readerConnected = false;
    close(serial);
}

std::string localIp() {
    ifaddrs* interfaces = nullptr;
    if (getifaddrs(&interfaces) != 0) return "";

    std::string result;
    for (ifaddrs* item = interfaces; item != nullptr; item = item->ifa_next) {
        if (item->ifa_addr == nullptr || item->ifa_addr->sa_family != AF_INET) continue;
        if (std::strcmp(item->ifa_name, "lo") == 0) continue;
        char address[INET_ADDRSTRLEN]{};
        auto* socketAddress = reinterpret_cast<sockaddr_in*>(item->ifa_addr);
        inet_ntop(AF_INET, &socketAddress->sin_addr, address, sizeof(address));
        result = address;
        break;
    }
    freeifaddrs(interfaces);
    return result;
}

std::string defaultGateway() {
    std::ifstream routes("/proc/net/route");
    std::string interface;
    unsigned long destination = 0;
    unsigned long gateway = 0;
    int flags = 0;
    std::string line;
    std::getline(routes, line);
    while (routes >> interface >> std::hex >> destination >> gateway >> std::dec >> flags) {
        if (destination != 0 || gateway == 0) continue;
        in_addr address{};
        address.s_addr = static_cast<in_addr_t>(gateway);
        char value[INET_ADDRSTRLEN]{};
        inet_ntop(AF_INET, &address, value, sizeof(value));
        return value;
    }
    return "";
}

bool validIpv4(const std::string& value) {
    static const std::regex pattern(R"(^((25[0-5])|(2[0-4][0-9])|(1[0-9]{2})|([1-9]?[0-9]))(\.((25[0-5])|(2[0-4][0-9])|(1[0-9]{2})|([1-9]?[0-9]))){3}$)");
    return std::regex_match(value, pattern);
}

std::string jsonValue(const std::string& body, const std::string& key) {
    const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]+)\\\"");
    std::smatch match;
    if (std::regex_search(body, match, pattern)) return match[1].str();
    return "";
}

int jsonInteger(const std::string& body, const std::string& key) {
    const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+)");
    std::smatch match;
    if (std::regex_search(body, match, pattern)) return std::atoi(match[1].str().c_str());
    return -1;
}

bool updateNetwork(const std::string& ip, const std::string& gateway) {
    if (!validIpv4(ip) || !validIpv4(gateway)) return false;
    std::ofstream netplan(NetplanFile);
    if (!netplan) return false;
    netplan << "network:\n"
            << "  version: 2\n"
            << "  ethernets:\n"
            << "    eth0:\n"
            << "      dhcp4: false\n"
            << "      addresses: [" << ip << "/24]\n"
            << "      routes:\n"
            << "        - to: 0.0.0.0/0\n"
            << "          via: " << gateway << "\n";
    netplan.close();
    return std::system("/usr/sbin/netplan apply") == 0;
}

std::string readFile(const std::string& path) {
    std::ifstream file(path);
    std::stringstream content;
    content << file.rdbuf();
    return content.str();
}

std::string response(const std::string& status, const std::string& contentType, const std::string& body) {
    std::ostringstream output;
    output << "HTTP/1.1 " << status << "\r\n"
           << "Content-Type: " << contentType << "\r\n"
           << "Content-Length: " << body.size() << "\r\n"
           << "Connection: close\r\n\r\n"
           << body;
    return output.str();
}

void handleClient(int client) {
    char buffer[8192]{};
    const ssize_t bytes = recv(client, buffer, sizeof(buffer) - 1, 0);
    if (bytes <= 0) {
        close(client);
        return;
    }

    std::string request(buffer, static_cast<size_t>(bytes));
    const auto lineEnd = request.find("\r\n");
    const std::string requestLine = request.substr(0, lineEnd);
    std::string body;

    if (requestLine.rfind("GET /api/get-ip ", 0) == 0) {
        body = "{\"ip\":\"" + localIp() + "\",\"gateway\":\"" + defaultGateway() + "\",\"status\":\"ok\"}";
        const auto result = response("200 OK", "application/json", body);
        send(client, result.c_str(), result.size(), 0);
    } else if (requestLine.rfind("GET /api/reader-info ", 0) == 0) {
        body = "{\"receiver_count\":" + std::to_string(ReceiverCount) + "}";
        const auto result = response("200 OK", "application/json", body);
        send(client, result.c_str(), result.size(), 0);
    } else if (requestLine.rfind("GET /api/status-log", 0) == 0) {
        int filterAntenna = 0;
        const auto query = requestLine.find("antenna=");
        if (query != std::string::npos) filterAntenna = std::atoi(requestLine.c_str() + query + 8);
        std::lock_guard<std::mutex> lock(antennaMutex);
        body = "{\"readings\":[";
        bool first = true;
        for (const auto& reading : readings) {
            if (filterAntenna != 0 && reading.antenna != filterAntenna) continue;
            if (!first) body += ",";
            first = false;
            body += "{\"timestamp\":\"" + reading.timestamp + "\",\"antenna\":" + std::to_string(reading.antenna) + ",\"epc\":\"" + reading.epc + "\",\"rssi\":" + std::to_string(reading.rssi) + ",\"count\":" + std::to_string(reading.count) + "}";
        }
        body += "]}";
        const auto result = response("200 OK", "application/json", body);
        send(client, result.c_str(), result.size(), 0);
    } else if (requestLine.rfind("POST /api/status-clear ", 0) == 0) {
        std::lock_guard<std::mutex> lock(antennaMutex);
        readings.clear();
        body = "{\"status\":\"success\",\"message\":\"Status log cleared\"}";
        const auto result = response("200 OK", "application/json", body);
        send(client, result.c_str(), result.size(), 0);
    } else if (requestLine.rfind("GET /api/antenna-status ", 0) == 0) {
        std::lock_guard<std::mutex> lock(antennaMutex);
        body = "{\"source\":\"" + std::string(readerConnected ? "reader" : "unavailable") + "\",\"antennas\":[";
        for (int antenna = 0; antenna < ReceiverCount; ++antenna) {
            if (antenna > 0) body += ",";
            body += "{\"id\":" + std::to_string(antenna + 1) + ",\"detected\":" + (antennaStates[antenna].detected ? "true" : "false") + "}";
        }
        body += "]}";
        const auto result = response("200 OK", "application/json", body);
        send(client, result.c_str(), result.size(), 0);
    } else if (requestLine.rfind("GET /api/scan-status ", 0) == 0) {
        body = "{\"scanning\":" + std::string(scanEnabled ? "true" : "false") + ",\"mode\":\"" + (bufferMode ? "buffer" : "normal") + "\",\"buffer_log_available\":" + std::string(bufferLogAvailable ? "true" : "false") + "}";
        const auto result = response("200 OK", "application/json", body);
        send(client, result.c_str(), result.size(), 0);
    } else if (requestLine.rfind("POST /api/antenna-power ", 0) == 0) {
        const auto bodyStart = request.find("\r\n\r\n");
        const std::string payload = bodyStart == std::string::npos ? "" : request.substr(bodyStart + 4);
        const int power = jsonInteger(payload, "power");
        if (power >= 0 && power <= 40) {
            requestedRfPower = power;
            body = "{\"status\":\"success\",\"message\":\"RF power update queued\",\"power\":" + std::to_string(power) + "}";
            const auto result = response("200 OK", "application/json", body);
            send(client, result.c_str(), result.size(), 0);
        } else {
            body = "{\"status\":\"error\",\"message\":\"Power must be between 0 and 40 dBm\"}";
            const auto result = response("400 Bad Request", "application/json", body);
            send(client, result.c_str(), result.size(), 0);
        }
    } else if (requestLine.rfind("GET /api/buffer-log", 0) == 0) {
        std::lock_guard<std::mutex> lock(antennaMutex);
        body = "{\"readings\":[";
        bool first = true;
        for (const auto& reading : bufferReadings) {
            if (!first) body += ",";
            first = false;
            body += "{\"timestamp\":\"" + reading.timestamp + "\",\"antenna\":" + std::to_string(reading.antenna) + ",\"epc\":\"" + reading.epc + "\",\"rssi\":" + std::to_string(reading.rssi) + ",\"count\":" + std::to_string(reading.count) + "}";
        }
        body += "]}";
        const auto result = response("200 OK", "application/json", body);
        send(client, result.c_str(), result.size(), 0);
    } else if (requestLine.rfind("POST /api/buffer-clear ", 0) == 0) {
        std::lock_guard<std::mutex> lock(antennaMutex);
        bufferReadings.clear();
        bufferLogAvailable = false;
        body = "{\"status\":\"success\",\"message\":\"Buffer log cleared\"}";
        const auto result = response("200 OK", "application/json", body);
        send(client, result.c_str(), result.size(), 0);
    } else if (requestLine.rfind("POST /api/scan-control ", 0) == 0) {
        const auto bodyStart = request.find("\r\n\r\n");
        const std::string payload = bodyStart == std::string::npos ? "" : request.substr(bodyStart + 4);
        const std::string action = jsonValue(payload, "action");
        const std::string mode = jsonValue(payload, "mode");
        if (action == "start" || action == "stop") {
            scanEnabled = action == "start";
            if (action == "start") {
                bufferMode = mode == "buffer";
                if (bufferMode) {
                    std::lock_guard<std::mutex> lock(antennaMutex);
                    bufferReadings.clear();
                    bufferLogAvailable = false;
                }
            } else {
                if (bufferMode) bufferLogAvailable = true;
                bufferMode = false;
            }
            queueEsp32Status(scanEnabled);
            body = "{\"status\":\"success\",\"scanning\":" + std::string(scanEnabled ? "true" : "false") + ",\"mode\":\"" + (bufferMode ? "buffer" : "normal") + "\"}";
            const auto result = response("200 OK", "application/json", body);
            send(client, result.c_str(), result.size(), 0);
        } else {
            body = "{\"status\":\"error\",\"message\":\"action must be start or stop\"}";
            const auto result = response("400 Bad Request", "application/json", body);
            send(client, result.c_str(), result.size(), 0);
        }
    } else if (requestLine.rfind("POST /api/update_network ", 0) == 0) {
        const auto bodyStart = request.find("\r\n\r\n");
        const std::string payload = bodyStart == std::string::npos ? "" : request.substr(bodyStart + 4);
        const std::string ip = jsonValue(payload, "ip");
        const std::string gateway = jsonValue(payload, "gateway");
        if (updateNetwork(ip, gateway)) {
            body = "{\"status\":\"success\",\"message\":\"Network updated\"}";
            const auto result = response("200 OK", "application/json", body);
            send(client, result.c_str(), result.size(), 0);
        } else {
            body = "{\"status\":\"error\",\"message\":\"Invalid network data or netplan failed\"}";
            const auto result = response("400 Bad Request", "application/json", body);
            send(client, result.c_str(), result.size(), 0);
        }
    } else if (requestLine.rfind("GET /assets/", 0) == 0) {
        const auto pathStart = request.find("GET /assets/") + 4;
        const auto pathEnd = request.find(' ', pathStart);
        const std::string assetPath = request.substr(pathStart, pathEnd - pathStart);
        if (assetPath.find("..") != std::string::npos) {
            body = "Not found";
            const auto result = response("404 Not Found", "text/plain; charset=utf-8", body);
            send(client, result.c_str(), result.size(), 0);
        } else {
            body = readFile("/var/www/rfid-web-cpp/" + assetPath);
            const auto result = response("200 OK", "image/png", body);
            send(client, result.c_str(), result.size(), 0);
        }
    } else if (requestLine.rfind("GET / ", 0) == 0 || requestLine.rfind("GET /index.html ", 0) == 0) {
        body = readFile("/var/www/rfid-web-cpp/index.html");
        const auto result = response("200 OK", "text/html; charset=utf-8", body);
        send(client, result.c_str(), result.size(), 0);
    } else {
        body = "Not found";
        const auto result = response("404 Not Found", "text/plain; charset=utf-8", body);
        send(client, result.c_str(), result.size(), 0);
    }
    close(client);
}
}

int main() {
    std::signal(SIGPIPE, SIG_IGN); // a dropped HTTP client or ESP32 Bluetooth link must not kill the process

    const int server = socket(AF_INET, SOCK_STREAM, 0);
    if (server < 0) return 1;

    int reuse = 1;
    setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(Port);

    if (bind(server, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0 || listen(server, 16) < 0) {
        close(server);
        return 1;
    }

    std::cout << "rfid-web-cpp listening on port " << Port << std::endl;
    std::thread reader(readerLoop);
    std::thread esp32(esp32Loop);
    while (true) {
        const int client = accept(server, nullptr, nullptr);
        if (client >= 0) handleClient(client);
    }
    readerRunning = false;
    esp32BridgeRunning = false;
    reader.join();
    esp32.join();
}