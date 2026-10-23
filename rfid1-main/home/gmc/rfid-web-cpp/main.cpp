// rfid-web-cpp - Pi-side backend for the RFID gate system.
//
// Threads:
//   readerLoop  - owns the RFID reader serial port: inventory per antenna,
//                 per-antenna RF power, tag probe/write jobs from the web UI.
//                 Reconnects automatically if the reader drops.
//   esp32Loop   - newline-delimited JSON link to the ESP32 panel (unchanged).
//   main        - HTTP server on port 80, one short-lived thread per request.
//
// Settings (per-antenna power, enabled antennas, write power, antenna
// positions for the 3D view) are saved to rfid-config.json next to the web
// files, so they survive a page refresh and a reboot.

#include <arpa/inet.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {
constexpr int ReceiverCount = 4;
constexpr const char* NetplanFile = "/etc/netplan/50-cloud-init.yaml";
constexpr std::array<const char*, 3> SerialDevices{"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0"};
// Wired RS-422 link to the ESP32 panel: newline-delimited JSON, both
// directions, at the same baud rate as the ESP32 firmware's kWireBaud.
// This is the Pi's onboard UART0 header pins (GPIO14/TXD0 = physical pin 8,
// GPIO15/RXD0 = physical pin 10), not a USB adapter - it needs enable_uart=1
// in /boot/firmware/config.txt and the serial console disabled.
constexpr std::array<const char*, 3> Esp32SerialDevices{"/dev/serial0", "/dev/ttyAMA0", "/dev/ttyUSB1"};
constexpr speed_t Esp32SerialBaud = B57600;
constexpr const char* Esp32DeviceId = "pi-rfid";

std::string envOr(const char* name, const std::string& fallback) {
    const char* value = std::getenv(name);
    return value && *value ? std::string(value) : fallback;
}

const std::string WebRoot = envOr("RFID_WEB_ROOT", "/var/www/rfid-web-cpp");
const std::string ConfigPath = envOr("RFID_CONFIG", WebRoot + "/rfid-config.json");
const int HttpPort = std::atoi(envOr("RFID_HTTP_PORT", "80").c_str());

// ---------------------------------------------------------------- state ----

struct AntennaState {
    bool detected = false;
    std::chrono::steady_clock::time_point lastSeen{};
};

struct Reading {
    int antenna = 0;
    std::string epc;
    int rssi = 0;
    int rssiMax = 0;
    int count = 0;
    std::string timestamp;
};

// Live per-tag, per-antenna signal used by the 3D view.
struct LiveAntenna {
    int rssi = 0;
    double avg = 0;   // exponential moving average of RSSI
    int64_t firstMs = 0;  // start of the current pass (reset after a 3 s gap)
    int64_t lastMs = 0;
    int hits = 0;
};
struct LiveTag {
    std::array<LiveAntenna, ReceiverCount> ant{};
    int64_t lastMs = 0;
};

struct Config {
    std::array<int, ReceiverCount> power{15, 15, 15, 15};
    std::array<bool, ReceiverCount> enabled{true, true, true, true};
    int writePower = 15;
    // x, y, z in metres. Default: gate A (ant 1,2) at z=0, gate B (ant 3,4) at z=3.
    std::array<std::array<double, 3>, ReceiverCount> pos{{{-1, 1, 0}, {1, 1, 0}, {-1, 1, 3}, {1, 1, 3}}};
    // Facing direction in degrees. yaw: top view, 0 = +Z (along the passage),
    // 90 = +X (right), -90 = -X (left). tilt: 0 = level, negative = down.
    std::array<double, ReceiverCount> yaw{90, -90, 90, -90};
    std::array<double, ReceiverCount> tilt{0, 0, 0, 0};
    std::array<double, ReceiverCount> beam{65, 65, 65, 65};  // beam width, degrees
    double levelSplit = 1.1;  // metres: tags above = upper level, below = lower (3D view)
    // Which channel to scan: -1 = follow the ESP32 channel switch, 0 = all
    // antennas, 1 = channel 1 (antennas 1-2), 2 = channel 2 (antennas 3-4).
    int gateMode = -1;
};

std::array<AntennaState, ReceiverCount> antennaStates;
std::mutex antennaMutex;  // guards antennaStates, readings, bufferReadings, liveTags
std::vector<Reading> readings;
std::vector<Reading> bufferReadings;
std::map<std::string, LiveTag> liveTags;

std::mutex configMutex;
Config config;

std::atomic<bool> readerRunning{true};
std::atomic<bool> readerConnected{false};
std::atomic<int> readerTypeCode{-1};
std::atomic<bool> scanEnabled{false};
std::atomic<bool> bufferMode{false};
std::atomic<bool> bufferLogAvailable{false};
std::atomic<bool> esp32BridgeRunning{true};
std::atomic<bool> esp32Connected{false};
// Channel reported by the ESP32 switch (GPIO25): 0 = unknown, 1 or 2.
std::atomic<int> esp32Gate{0};
std::atomic<int64_t> esp32GateMs{0};  // last time the ESP32 told us its channel
std::array<std::atomic<int>, ReceiverCount> appliedPower{};  // last power the reader ACKed per antenna, -1 = unknown

// ---------------------------------------------------------------- GPIO -----
// Two panel indicator LEDs driven directly from the Pi's own GPIO.
// NOTE: this register layout is BCM2711 (Pi 4). On a Pi 5 the GPIO lives on
// the RP1 chip and /dev/gpiomem does not exist, so the LEDs are skipped.
constexpr uint8_t PiReadyLedPin = 23;
constexpr uint8_t PiEsp32LinkLedPin = 24;
volatile uint32_t* gGpioRegs = nullptr;

bool gpioInit() {
    const int fd = open("/dev/gpiomem", O_RDWR | O_SYNC);
    if (fd < 0) return false;
    void* mapped = mmap(nullptr, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (mapped == MAP_FAILED) return false;
    gGpioRegs = static_cast<volatile uint32_t*>(mapped);
    return true;
}

void gpioSetOutput(uint8_t pin) {
    if (!gGpioRegs) return;
    const uint32_t regIndex = pin / 10;
    const uint32_t shift = (pin % 10) * 3;
    gGpioRegs[regIndex] = (gGpioRegs[regIndex] & ~(0x7u << shift)) | (0x1u << shift);
}

void gpioWrite(uint8_t pin, bool high) {
    if (!gGpioRegs) return;
    constexpr uint32_t GpsetWord = 7;
    constexpr uint32_t GpclrWord = 10;
    gGpioRegs[(high ? GpsetWord : GpclrWord) + pin / 32] = (1u << (pin % 32));
}

// ---------------------------------------------------------------- utils ----

uint16_t checksum(const unsigned char* bytes, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408) : static_cast<uint16_t>(crc >> 1);
        }
    }
    return crc;
}
uint16_t checksum(const std::vector<unsigned char>& bytes) { return checksum(bytes.data(), bytes.size()); }

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

std::string timestampNow() {
    const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    char buffer[32]{};
    std::tm local{};
    localtime_r(&now, &local);
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &local);
    return buffer;
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

bool hexToBytes(std::string hex, std::vector<unsigned char>& out) {
    hex.erase(std::remove_if(hex.begin(), hex.end(), [](char c) { return c == ' ' || c == '-' || c == ':'; }), hex.end());
    if (hex.size() % 2 != 0) return false;
    out.clear();
    for (size_t i = 0; i < hex.size(); i += 2) {
        if (!std::isxdigit(static_cast<unsigned char>(hex[i])) || !std::isxdigit(static_cast<unsigned char>(hex[i + 1]))) return false;
        out.push_back(static_cast<unsigned char>(std::stoi(hex.substr(i, 2), nullptr, 16)));
    }
    return true;
}

std::string upper(std::string s) {
    for (auto& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

// Tiny JSON helpers - the payloads here are small and flat.
std::string jsonValue(const std::string& body, const std::string& key) {
    const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
    std::smatch match;
    if (std::regex_search(body, match, pattern)) return match[1].str();
    return "";
}

int jsonInteger(const std::string& body, const std::string& key, int fallback = -1) {
    const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+)");
    std::smatch match;
    if (std::regex_search(body, match, pattern)) return std::atoi(match[1].str().c_str());
    return fallback;
}

bool jsonHasKey(const std::string& body, const std::string& key) {
    return body.find("\"" + key + "\"") != std::string::npos;
}

std::vector<double> jsonNumberArray(const std::string& body, const std::string& key) {
    std::vector<double> out;
    const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*\\[([^\\]]*)\\]");
    std::smatch match;
    if (!std::regex_search(body, match, pattern)) return out;
    std::string inner = match[1].str();
    for (auto& c : inner) if (c == ',') c = ' ';
    std::istringstream stream(inner);
    std::string token;
    while (stream >> token) {
        if (token == "true") out.push_back(1);
        else if (token == "false") out.push_back(0);
        else out.push_back(std::atof(token.c_str()));
    }
    return out;
}

// --------------------------------------------------------------- config ----

std::string configToJson(const Config& c) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(2);
    out << "{\"powers\":[";
    for (int i = 0; i < ReceiverCount; ++i) out << (i ? "," : "") << c.power[i];
    out << "],\"enabled\":[";
    for (int i = 0; i < ReceiverCount; ++i) out << (i ? "," : "") << (c.enabled[i] ? "true" : "false");
    out << "],\"write_power\":" << c.writePower << ",\"positions\":[";
    for (int i = 0; i < ReceiverCount; ++i)
        for (int k = 0; k < 3; ++k) out << ((i || k) ? "," : "") << c.pos[i][k];
    const auto arr = [&](const char* key, const std::array<double, ReceiverCount>& v) {
        out << "],\"" << key << "\":[";
        for (int i = 0; i < ReceiverCount; ++i) out << (i ? "," : "") << v[i];
    };
    arr("yaw", c.yaw);
    arr("tilt", c.tilt);
    arr("beam", c.beam);
    out << "],\"level_split\":" << c.levelSplit << ",\"gate_mode\":" << c.gateMode << "}";
    return out.str();
}

// Applies whichever keys are present in body. Returns false on a bad value.
bool applyConfigJson(Config& c, const std::string& body, std::string& error) {
    if (jsonHasKey(body, "powers")) {
        const auto p = jsonNumberArray(body, "powers");
        if (p.size() != ReceiverCount) { error = "powers must have 4 values"; return false; }
        for (int i = 0; i < ReceiverCount; ++i) {
            if (p[i] < 0 || p[i] > 33) { error = "power must be 0-33 dBm"; return false; }
            c.power[i] = static_cast<int>(p[i]);
        }
    }
    if (jsonHasKey(body, "enabled")) {
        const auto e = jsonNumberArray(body, "enabled");
        if (e.size() != ReceiverCount) { error = "enabled must have 4 values"; return false; }
        for (int i = 0; i < ReceiverCount; ++i) c.enabled[i] = e[i] != 0;
    }
    if (jsonHasKey(body, "write_power")) {
        const int w = jsonInteger(body, "write_power");
        if (w < 0 || w > 33) { error = "write_power must be 0-33 dBm"; return false; }
        c.writePower = w;
    }
    if (jsonHasKey(body, "positions")) {
        const auto p = jsonNumberArray(body, "positions");
        if (p.size() != ReceiverCount * 3) { error = "positions must have 12 numbers"; return false; }
        for (int i = 0; i < ReceiverCount; ++i)
            for (int k = 0; k < 3; ++k) c.pos[i][k] = std::max(-50.0, std::min(50.0, p[i * 3 + k]));
    }
    const auto angles = [&](const char* key, std::array<double, ReceiverCount>& target, double lo, double hi) {
        if (!jsonHasKey(body, key)) return true;
        const auto v = jsonNumberArray(body, key);
        if (v.size() != ReceiverCount) { error = std::string(key) + " must have 4 values"; return false; }
        for (int i = 0; i < ReceiverCount; ++i) target[i] = std::max(lo, std::min(hi, v[i]));
        return true;
    };
    if (!angles("yaw", c.yaw, -360, 360)) return false;
    if (!angles("tilt", c.tilt, -90, 90)) return false;
    if (!angles("beam", c.beam, 10, 180)) return false;
    if (jsonHasKey(body, "gate_mode")) {
        const int g = jsonInteger(body, "gate_mode", -2);
        if (g < -1 || g > 2) { error = "gate_mode must be -1, 0, 1 or 2"; return false; }
        c.gateMode = g;
    }
    if (jsonHasKey(body, "level_split")) {
        const std::regex pattern("\"level_split\"\\s*:\\s*(-?[0-9.]+)");
        std::smatch match;
        if (!std::regex_search(body, match, pattern)) { error = "level_split must be a number"; return false; }
        c.levelSplit = std::max(0.0, std::min(10.0, std::atof(match[1].str().c_str())));
    }
    return true;
}

std::string readFile(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    std::stringstream content;
    content << file.rdbuf();
    return content.str();
}

void loadConfig() {
    const std::string text = readFile(ConfigPath);
    if (text.empty()) {
        std::cerr << "Config: " << ConfigPath << " not found, using defaults" << std::endl;
        return;
    }
    std::lock_guard<std::mutex> lock(configMutex);
    std::string error;
    Config loaded = config;
    if (applyConfigJson(loaded, text, error)) {
        config = loaded;
        std::cerr << "Config loaded: " << configToJson(config) << std::endl;
    } else {
        std::cerr << "Config: ignoring " << ConfigPath << " (" << error << ")" << std::endl;
    }
}

bool saveConfigLocked() {
    const std::string tmp = ConfigPath + ".tmp";
    {
        std::ofstream out(tmp, std::ios::trunc);
        if (!out) return false;
        out << configToJson(config) << "\n";
        if (!out) return false;
    }
    return std::rename(tmp.c_str(), ConfigPath.c_str()) == 0;
}

Config configSnapshot() {
    std::lock_guard<std::mutex> lock(configMutex);
    return config;
}

// ---------------------------------------------------------- ESP32 link -----

std::mutex esp32QueueMutex;
std::deque<std::string> esp32Queue;

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

void queueEsp32Line(const std::string& json) {
    std::lock_guard<std::mutex> lock(esp32QueueMutex);
    if (esp32Queue.size() >= 128) esp32Queue.pop_front();
    esp32Queue.push_back(json + "\n");
}

void queueEsp32Status(bool scanning) { queueEsp32Line(buildStatusJson(scanning)); }
void queueEsp32Reading(const Reading& reading) { queueEsp32Line(buildScanResultJson(reading)); }
void queueEsp32Heartbeat() { queueEsp32Line("{\"type\":\"ping\",\"ts\":" + std::to_string(epochMillisNow()) + "}"); }

bool applyScanControl(const std::string& action, const std::string& mode) {
    if (action != "start" && action != "stop") return false;
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
    return true;
}

void resetReadings() {
    std::lock_guard<std::mutex> lock(antennaMutex);
    readings.clear();
    bufferReadings.clear();
    liveTags.clear();
    bufferLogAvailable = false;
}

void setEsp32Gate(int gate, bool announce) {
    if (gate != 1 && gate != 2) return;
    esp32GateMs = epochMillisNow();
    if (esp32Gate.exchange(gate) != gate || announce)
        std::cerr << "ESP32 panel: channel " << gate << " (antennas " << (gate == 1 ? "1-2" : "3-4") << ")" << std::endl;
}

// Channel actually being scanned right now: 0 = all antennas, 1 or 2.
int effectiveGate(int gateMode) {
    if (gateMode >= 0) return gateMode;
    // Follow the ESP32; forget its channel if it went quiet (link down).
    if (epochMillisNow() - esp32GateMs.load() > 5000) return 0;
    return esp32Gate.load();
}
int channelOfAntenna(int antenna) { return antenna <= 2 ? 1 : 2; }

void processEsp32Line(const std::string& line) {
    const std::string type = jsonValue(line, "type");
    if (type == "pong") {
        // Newer ESP32 firmware repeats its current channel in every pong, so
        // the Pi learns it at boot / after a reconnect without a switch flip.
        setEsp32Gate(jsonInteger(line, "gate", 0), false);
        return;
    }
    if (type != "scan_control") return;
    const std::string action = jsonValue(line, "action");
    if (action == "reset") {
        resetReadings();
        queueEsp32Status(scanEnabled);
        std::cerr << "ESP32 panel: reset" << std::endl;
    } else if (action == "select") {
        setEsp32Gate(jsonInteger(line, "gate", 0), true);
    } else if (action == "export") {
        std::cerr << "ESP32 panel: export requested" << std::endl;
    } else {
        applyScanControl(action, "");
    }
}

int openSerialRaw(const char* device, speed_t baud, bool sync) {
    const int serial = open(device, O_RDWR | O_NOCTTY | (sync ? O_SYNC : 0));
    if (serial < 0) return -1;
    termios settings{};
    tcgetattr(serial, &settings);
    cfmakeraw(&settings);
    settings.c_cflag = CS8 | CREAD | CLOCAL;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 5;
    cfsetispeed(&settings, baud);
    cfsetospeed(&settings, baud);
    tcsetattr(serial, TCSANOW, &settings);
    tcflush(serial, TCIOFLUSH);
    return serial;
}

int openEsp32Serial() {
    if (const char* configured = std::getenv("ESP32_SERIAL_DEVICE")) {
        const int serial = openSerialRaw(configured, Esp32SerialBaud, true);
        if (serial >= 0) std::cerr << "ESP32 link: opened " << configured << " (from ESP32_SERIAL_DEVICE)" << std::endl;
        return serial;
    }
    for (const auto device : Esp32SerialDevices) {
        const int serial = openSerialRaw(device, Esp32SerialBaud, true);
        if (serial < 0) continue;
        std::cerr << "ESP32 link: opened " << device << std::endl;
        return serial;
    }
    return -1;
}

void esp32Loop() {
    bool loggedMissing = false;
    while (esp32BridgeRunning) {
        const int serial = openEsp32Serial();
        if (serial < 0) {
            if (!loggedMissing) std::cerr << "ESP32 link unavailable: no serial device found (retrying quietly)" << std::endl;
            loggedMissing = true;
            std::this_thread::sleep_for(std::chrono::seconds(2));
            continue;
        }
        loggedMissing = false;
        esp32Connected = true;
        gpioWrite(PiEsp32LinkLedPin, true);
        queueEsp32Status(scanEnabled);

        auto lastHeartbeat = std::chrono::steady_clock::now();
        std::string rxBuffer;
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
                const auto written = write(serial, line.data(), line.size());
                if (written != static_cast<ssize_t>(line.size())) {
                    std::cerr << "ESP32 link write failed; reopening" << std::endl;
                    linkOk = false;
                    break;
                }
            }

            fd_set readSet;
            FD_ZERO(&readSet);
            FD_SET(serial, &readSet);
            timeval noWait{0, 0};
            if (select(serial + 1, &readSet, nullptr, nullptr, &noWait) > 0) {
                char readBuffer[512];
                const ssize_t received = read(serial, readBuffer, sizeof(readBuffer));
                if (received > 0) {
                    rxBuffer.append(readBuffer, static_cast<size_t>(received));
                    size_t newlinePos;
                    while ((newlinePos = rxBuffer.find('\n')) != std::string::npos) {
                        std::string commandLine = rxBuffer.substr(0, newlinePos);
                        rxBuffer.erase(0, newlinePos + 1);
                        if (!commandLine.empty() && commandLine.back() == '\r') commandLine.pop_back();
                        processEsp32Line(commandLine);
                    }
                    if (rxBuffer.size() > 4096) rxBuffer.clear();
                } else {
                    std::cerr << "ESP32 link read failed; reopening" << std::endl;
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
        gpioWrite(PiEsp32LinkLedPin, false);
        close(serial);
        if (esp32BridgeRunning) std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

// ------------------------------------------------------- reader protocol ---
// Frame: Len Adr Cmd Data... CRC_LSB CRC_MSB  (Len counts everything after itself)

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

// Reader type 0x75 (UHFReader288M path): Q=4, session 0, no mask, target A, antenna, scantime 2 s.
std::vector<unsigned char> inventoryCommand(unsigned char antennaCode) {
    return commandFrame(0x01, {0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, antennaCode, 0x14});
}
std::vector<unsigned char> reader18InventoryCommand() { return commandFrame(0x01, {}); }

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

// Returns false on timeout, bad length or bad CRC (input is flushed on CRC error
// so the next frame starts clean).
bool readFrame(int serial, std::vector<unsigned char>& frame, int timeoutMs) {
    unsigned char length = 0;
    if (!readExact(serial, &length, 1, timeoutMs)) return false;
    if (length < 4) { tcflush(serial, TCIFLUSH); return false; }
    frame.resize(static_cast<size_t>(length) + 1);
    frame[0] = length;
    if (!readExact(serial, frame.data() + 1, length, timeoutMs)) return false;
    const uint16_t crc = checksum(frame.data(), frame.size() - 2);
    if ((crc & 0xFF) != frame[frame.size() - 2] || (crc >> 8) != frame[frame.size() - 1]) {
        std::cerr << "RFID frame CRC mismatch: " << hexDump(frame) << std::endl;
        tcflush(serial, TCIFLUSH);
        return false;
    }
    return true;
}

struct TagHit {
    int antenna = 0;
    std::string epc;
    int rssi = 0;
};

// Parses one inventory response frame. Returns the status byte.
int parseInventoryFrame(const std::vector<unsigned char>& frame, bool reader18, int requestedAntenna, std::vector<TagHit>& hits) {
    if (frame.size() < 6) return -1;
    const int status = frame[3];
    if (status < 1 || status > 4) return status;
    if (reader18) {
        // Len Adr Cmd Status Num [EpcLen EPC]... CRC
        const int tagCount = frame[4];
        size_t cursor = 5;
        for (int tag = 0; tag < tagCount && cursor < frame.size() - 2; ++tag) {
            const size_t tagLength = frame[cursor++];
            if (cursor + tagLength > frame.size() - 2) break;
            hits.push_back({requestedAntenna, bytesToHex(frame.data() + cursor, tagLength), 0});
            cursor += tagLength;
        }
        return status;
    }
    // Len Adr Cmd Status Ant Num [EpcLen EPC RSSI]... CRC
    const unsigned char antennaMask = frame[4];
    int antenna = antennaMask == 1 ? 1 : antennaMask == 2 ? 2 : antennaMask == 4 ? 3 : antennaMask == 8 ? 4 : requestedAntenna;
    const int tagCount = frame[5];
    size_t cursor = 6;
    for (int tag = 0; tag < tagCount && cursor < frame.size() - 2; ++tag) {
        const size_t tagLength = frame[cursor++] & 0x7F;  // bit7 = TID appended flag
        if (cursor + tagLength > frame.size() - 2) break;
        const std::string epc = bytesToHex(frame.data() + cursor, tagLength);
        cursor += tagLength;
        int rssi = 0;
        if (cursor < frame.size() - 2) rssi = frame[cursor++];
        hits.push_back({antenna, epc, rssi});
    }
    return status;
}

void recordHit(const TagHit& hit) {
    if (hit.antenna < 1 || hit.antenna > ReceiverCount || hit.epc.empty()) return;
    const int64_t nowMs = epochMillisNow();
    std::lock_guard<std::mutex> lock(antennaMutex);
    antennaStates[hit.antenna - 1].detected = true;
    antennaStates[hit.antenna - 1].lastSeen = std::chrono::steady_clock::now();

    auto& targetReadings = bufferMode ? bufferReadings : readings;
    const auto existing = std::find_if(targetReadings.begin(), targetReadings.end(), [&](const Reading& r) {
        return r.antenna == hit.antenna && r.epc == hit.epc;
    });
    if (existing != targetReadings.end()) {
        ++existing->count;
        existing->rssi = hit.rssi;
        existing->rssiMax = std::max(existing->rssiMax, hit.rssi);
        existing->timestamp = timestampNow();
        queueEsp32Reading(*existing);
    } else {
        targetReadings.push_back({hit.antenna, hit.epc, hit.rssi, hit.rssi, 1, timestampNow()});
        queueEsp32Reading(targetReadings.back());
    }
    const size_t maxReadings = bufferMode ? 5000 : 500;
    const size_t trimCount = bufferMode ? 500 : 100;
    if (targetReadings.size() > maxReadings) targetReadings.erase(targetReadings.begin(), targetReadings.begin() + trimCount);

    auto& tag = liveTags[hit.epc];
    auto& a = tag.ant[hit.antenna - 1];
    const bool fresh = a.lastMs == 0 || nowMs - a.lastMs > 3000;
    a.avg = fresh ? hit.rssi : a.avg * 0.6 + hit.rssi * 0.4;
    if (fresh) a.firstMs = nowMs;
    a.rssi = hit.rssi;
    a.lastMs = nowMs;
    ++a.hits;
    tag.lastMs = nowMs;
}

void pruneLiveTags() {
    const int64_t cutoff = epochMillisNow() - 120000;
    std::lock_guard<std::mutex> lock(antennaMutex);
    for (auto it = liveTags.begin(); it != liveTags.end();) {
        if (it->second.lastMs < cutoff) it = liveTags.erase(it);
        else ++it;
    }
}

// --------------------------------------------------------- tag jobs -------

struct TagJobResult {
    bool ok = false;
    std::string message;
    std::vector<TagHit> seen;  // aggregated: strongest RSSI per EPC
    std::string readBack;
};

struct TagJob {
    std::string kind;     // "probe" or "write"
    int antenna = 1;      // 1..4
    int power = -1;       // dBm, -1 = config write power
    std::string target;   // EPC of the tag to write (empty = only tag in field)
    std::string mem;      // "epc" or "user"
    int wordPtr = 0;      // user memory start word
    std::string data;     // hex
    std::string password; // 8 hex chars
    std::promise<TagJobResult> promise;
};

std::mutex jobMutex;
std::shared_ptr<TagJob> pendingJob;

std::string tagErrorText(int code) {
    switch (code) {
        case 0x00: return "other error";
        case 0x03: return "memory overrun / wrong length";
        case 0x04: return "memory locked";
        case 0x0B: return "insufficient power - move the tag closer or raise write power";
        case 0x0F: return "non-specific tag error";
        default: return "tag error 0x" + bytesToHex(reinterpret_cast<const unsigned char*>(&code), 1);
    }
}

std::string statusText(const std::vector<unsigned char>& response) {
    if (response.size() < 4) return "no response";
    const int status = response[3];
    switch (status) {
        case 0x00: return "ok";
        case 0xFA: return "tag found but communication failed - try again / closer";
        case 0xFB: return "no tag in field";
        case 0xFC: return response.size() > 6 ? tagErrorText(response[4]) : "tag error";
        case 0xFD: return "command length error";
        case 0xFE: return "illegal command";
        case 0xFF: return "parameter error";
        default: {
            const unsigned char s = static_cast<unsigned char>(status);
            return "reader status 0x" + bytesToHex(&s, 1);
        }
    }
}

class Reader {
public:
    explicit Reader(int fd) : serial_(fd) {}

    // Send a command and wait for its reply frame.
    bool transact(unsigned char command, const std::vector<unsigned char>& data, std::vector<unsigned char>& response, int timeoutMs = 1500, bool log = true) {
        const auto frame = commandFrame(command, data);
        tcflush(serial_, TCIFLUSH);
        if (log) std::cerr << "RFID command: " << hexDump(frame) << std::endl;
        if (write(serial_, frame.data(), frame.size()) != static_cast<ssize_t>(frame.size())) {
            ioError_ = true;
            return false;
        }
        for (int attempt = 0; attempt < 4; ++attempt) {
            if (!readFrame(serial_, response, timeoutMs)) return false;
            if (response.size() > 3 && response[2] == command) {
                if (log) std::cerr << "RFID response: " << hexDump(response) << std::endl;
                return true;
            }
        }
        return false;
    }

    bool configure(unsigned char command, const std::vector<unsigned char>& data, bool log = true) {
        std::vector<unsigned char> response;
        if (!transact(command, data, response, 1500, log)) {
            std::cerr << "RFID command timeout: 0x" << std::hex << static_cast<int>(command) << std::dec << std::endl;
            return false;
        }
        return response[3] == 0;
    }

    bool setPower(int dbm) {
        if (dbm == currentPower_) return true;
        // Only log the first few power commands; with different powers per
        // antenna this runs on every antenna switch.
        const bool ok = configure(0x2F, {static_cast<unsigned char>(dbm)}, powerLogBudget_ > 0);
        if (powerLogBudget_ > 0) --powerLogBudget_;
        // Remember it even on failure so a reader that rejects this value
        // doesn't get the command re-sent before every single inventory.
        currentPower_ = dbm;
        lastPowerOk_ = ok;
        if (!ok) std::cerr << "RFID power " << dbm << " dBm not acknowledged" << std::endl;
        return ok;
    }
    bool lastPowerOk() const { return lastPowerOk_; }
    void forgetPower() { currentPower_ = -1; }

    // One inventory round on one antenna; collects all response frames.
    std::vector<TagHit> inventory(int antenna, int timeoutMs) {
        std::vector<TagHit> hits;
        const auto command = reader18_ ? reader18InventoryCommand() : inventoryCommand(static_cast<unsigned char>(0x80 + antenna - 1));
        tcflush(serial_, TCIFLUSH);
        if (write(serial_, command.data(), command.size()) != static_cast<ssize_t>(command.size())) {
            ioError_ = true;
            return hits;
        }
        for (int frames = 0; frames < 32; ++frames) {
            std::vector<unsigned char> frame;
            if (!readFrame(serial_, frame, timeoutMs)) break;
            if (frame.size() < 4 || frame[2] != 0x01) continue;
            const int status = parseInventoryFrame(frame, reader18_, antenna, hits);
            if (status == 0x03) continue;  // more frames follow
            if (status != 0x01 && status != 0x02 && status != 0x04 && status != 0xFB)
                std::cerr << "RFID inventory status: " << hexDump(frame) << std::endl;
            break;
        }
        return hits;
    }

    bool init() {
        // Match the legacy Python defaults: US band, steps 0..48.
        configure(0x22, {0x30, 0x80});
        std::vector<unsigned char> info;
        if (!transact(0x21, {}, info, 5000)) {
            std::cerr << "RFID reader-info timeout: no response from serial device" << std::endl;
            return false;
        }
        if (info.size() < 7 || info[3] != 0) {
            std::cerr << "RFID reader-info returned an invalid status" << std::endl;
            return false;
        }
        readerTypeCode = info[6];
        reader18_ = info[6] == 0x09;
        std::cerr << "RFID reader type: 0x" << std::hex << static_cast<int>(info[6]) << std::dec << std::endl;
        const bool workMode = reader18_ ? configure(0x35, {0, 0, 0, 0, 0, 0}) : configure(0x76, {0});
        if (!workMode) std::cerr << "RFID work-mode command was not acknowledged; continuing with reader default mode." << std::endl;
        return true;
    }

    bool ioError() const { return ioError_; }
    bool reader18() const { return reader18_; }

    TagJobResult runJob(TagJob& job) {
        TagJobResult result;
        const Config cfg = configSnapshot();
        const int power = job.power >= 0 ? job.power : cfg.writePower;
        setPower(power);

        // 1) See which tags are in front of the chosen antenna (3 rounds).
        std::map<std::string, TagHit> seen;
        for (int round = 0; round < 3; ++round) {
            for (const auto& hit : inventory(job.antenna, 600)) {
                auto it = seen.find(hit.epc);
                if (it == seen.end() || hit.rssi > it->second.rssi) seen[hit.epc] = hit;
            }
        }
        for (const auto& kv : seen) result.seen.push_back(kv.second);
        std::sort(result.seen.begin(), result.seen.end(), [](const TagHit& a, const TagHit& b) { return a.rssi > b.rssi; });

        if (job.kind == "probe") {
            result.ok = true;
            result.message = result.seen.empty() ? "no tag found" : std::to_string(result.seen.size()) + " tag(s) found";
            return result;
        }

        // 2) Pick the target tag.
        std::string target = upper(job.target);
        if (target.empty()) {
            if (result.seen.empty()) { result.message = "no tag found at antenna " + std::to_string(job.antenna); return result; }
            if (result.seen.size() > 1) { result.message = "more than one tag in field - pick the target EPC first"; return result; }
            target = result.seen.front().epc;
        } else if (!seen.count(target)) {
            result.message = "target tag " + target + " not seen at antenna " + std::to_string(job.antenna);
            return result;
        }

        std::vector<unsigned char> targetBytes, newData, pwd;
        if (!hexToBytes(target, targetBytes) || targetBytes.empty() || targetBytes.size() % 2) { result.message = "bad target EPC"; return result; }
        if (!hexToBytes(job.password.empty() ? "00000000" : job.password, pwd) || pwd.size() != 4) { result.message = "password must be 8 hex digits"; return result; }
        if (!hexToBytes(job.data, newData) || newData.empty()) { result.message = "data must be hex"; return result; }
        if (newData.size() % 2) { result.message = "data must be whole words (multiple of 4 hex digits)"; return result; }

        unsigned char mem = 1;
        int wordPtr = 2;
        std::vector<unsigned char> payload = newData;
        if (job.mem == "user") {
            mem = 3;
            wordPtr = std::max(0, job.wordPtr);
        } else {
            if (newData.size() > 62) { result.message = "EPC too long (max 496 bit)"; return result; }
            if (newData.size() != targetBytes.size()) {
                // Length changes: rewrite the PC word too (EPC length in words, bits 15..11).
                const uint16_t pc = static_cast<uint16_t>((newData.size() / 2) << 11);
                payload.insert(payload.begin(), {static_cast<unsigned char>(pc >> 8), static_cast<unsigned char>(pc & 0xFF)});
                wordPtr = 1;
            }
        }

        // 3) Write: 0x03 WNum ENum EPC Mem WordPtr Data Pwd
        std::vector<unsigned char> data;
        data.push_back(static_cast<unsigned char>(payload.size() / 2));
        data.push_back(static_cast<unsigned char>(targetBytes.size() / 2));
        data.insert(data.end(), targetBytes.begin(), targetBytes.end());
        data.push_back(mem);
        data.push_back(static_cast<unsigned char>(wordPtr));
        data.insert(data.end(), payload.begin(), payload.end());
        data.insert(data.end(), pwd.begin(), pwd.end());

        const std::string newEpc = bytesToHex(newData.data(), newData.size());
        std::string lastError;
        for (int attempt = 1; attempt <= 3; ++attempt) {
            std::vector<unsigned char> response;
            const bool replied = transact(0x03, data, response, 2500);
            const bool writeOk = replied && response[3] == 0;
            lastError = replied ? statusText(response) : "no reply from reader";

            if (job.mem == "user") {
                if (!writeOk) continue;
                // Read back: 0x02 ENum EPC Mem WordPtr Num Pwd
                std::vector<unsigned char> readCmd;
                readCmd.push_back(static_cast<unsigned char>(targetBytes.size() / 2));
                readCmd.insert(readCmd.end(), targetBytes.begin(), targetBytes.end());
                readCmd.push_back(3);
                readCmd.push_back(static_cast<unsigned char>(wordPtr));
                readCmd.push_back(static_cast<unsigned char>(newData.size() / 2));
                readCmd.insert(readCmd.end(), pwd.begin(), pwd.end());
                std::vector<unsigned char> readResp;
                if (transact(0x02, readCmd, readResp, 2500) && readResp[3] == 0 && readResp.size() >= 6 + newData.size()) {
                    result.readBack = bytesToHex(readResp.data() + 4, readResp.size() - 6);
                    result.ok = upper(result.readBack) == newEpc;
                    result.message = result.ok ? "user memory written and verified" : "written but read-back differs";
                } else {
                    result.ok = true;
                    result.message = "written (reader ACK) - read-back not available";
                }
                return result;
            }

            // EPC: verify by inventory - the tag must now answer with the new EPC.
            bool newSeen = false, oldSeen = false;
            for (int round = 0; round < 3 && !newSeen; ++round) {
                for (const auto& hit : inventory(job.antenna, 600)) {
                    if (hit.epc == newEpc) newSeen = true;
                    if (hit.epc == target) oldSeen = true;
                }
            }
            if (newSeen) {
                result.ok = true;
                result.message = "written and verified: " + target + " -> " + newEpc;
                return result;
            }
            if (!oldSeen) {
                result.message = "tag left the field during write (" + lastError + ")";
                return result;
            }
            std::cerr << "RFID write attempt " << attempt << " failed: " << lastError << std::endl;
        }
        result.message = "write failed: " + lastError;
        return result;
    }

private:
    int serial_;
    bool reader18_ = false;
    bool ioError_ = false;
    int currentPower_ = -1;
    bool lastPowerOk_ = true;
    int powerLogBudget_ = 8;
};

int openReaderSerial(std::string& deviceName) {
    if (const char* configured = std::getenv("RFID_SERIAL_DEVICE")) {
        deviceName = configured;
        return openSerialRaw(configured, B57600, false);
    }
    for (const auto device : SerialDevices) {
        const int serial = openSerialRaw(device, B57600, false);
        if (serial >= 0) {
            deviceName = device;
            return serial;
        }
    }
    return -1;
}

void failPendingJob(const std::string& message) {
    std::shared_ptr<TagJob> job;
    {
        std::lock_guard<std::mutex> lock(jobMutex);
        job.swap(pendingJob);
    }
    if (job) {
        TagJobResult result;
        result.message = message;
        job->promise.set_value(result);
    }
}

void readerLoop() {
    bool loggedMissing = false;
    while (readerRunning) {
        std::string deviceName;
        const int serial = openReaderSerial(deviceName);
        if (serial < 0) {
            if (!loggedMissing) std::cerr << "RFID serial unavailable (retrying every 3 s)" << std::endl;
            loggedMissing = true;
            failPendingJob("reader not connected");
            std::this_thread::sleep_for(std::chrono::seconds(3));
            continue;
        }
        loggedMissing = false;
        std::cerr << "RFID serial opened: " << deviceName << std::endl;

        Reader reader(serial);
        if (!reader.init()) {
            close(serial);
            failPendingJob("reader not responding");
            std::this_thread::sleep_for(std::chrono::seconds(3));
            continue;
        }
        readerConnected = true;
        for (auto& p : appliedPower) p = -1;
        int pruneCounter = 0;

        while (readerRunning && !reader.ioError()) {
            std::shared_ptr<TagJob> job;
            {
                std::lock_guard<std::mutex> lock(jobMutex);
                job.swap(pendingJob);
            }
            if (job) {
                job->promise.set_value(reader.runJob(*job));
                continue;
            }

            if (!scanEnabled) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }

            const Config cfg = configSnapshot();
            const int gate = effectiveGate(cfg.gateMode);
            const int timeout = bufferMode ? 180 : 800;
            const int antennaPasses = reader.reader18() ? 1 : ReceiverCount;
            int scanned = 0;
            for (int index = 0; index < antennaPasses; ++index) {
                if (!reader.reader18() && !cfg.enabled[index]) continue;
                if (!reader.reader18() && gate != 0 && channelOfAntenna(index + 1) != gate) continue;  // other channel
                const int power = cfg.power[index];
                reader.setPower(power);
                appliedPower[index] = reader.lastPowerOk() ? power : -1;
                ++scanned;
                for (const auto& hit : reader.inventory(index + 1, timeout)) {
                    // the reader reports the antenna itself; drop anything from the other channel
                    if (gate != 0 && channelOfAntenna(hit.antenna) != gate) continue;
                    recordHit(hit);
                }
            }
            if (scanned == 0) std::this_thread::sleep_for(std::chrono::milliseconds(100));
            {
                std::lock_guard<std::mutex> lock(antennaMutex);
                const auto now = std::chrono::steady_clock::now();
                for (auto& state : antennaStates)
                    state.detected = state.lastSeen.time_since_epoch().count() != 0 && now - state.lastSeen < std::chrono::seconds(3);
            }
            if (++pruneCounter >= 50) {
                pruneCounter = 0;
                pruneLiveTags();
            }
        }
        readerConnected = false;
        close(serial);
        std::cerr << "RFID reader link lost; reconnecting" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

// ------------------------------------------------------------- network ----

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
    unsigned long destination = 0, gateway = 0;
    int flags = 0;
    std::string line;
    std::getline(routes, line);
    while (std::getline(routes, line)) {
        std::istringstream row(line);
        if (!(row >> interface >> std::hex >> destination >> gateway >> std::dec >> flags)) continue;
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

// ---------------------------------------------------------------- HTTP ----

struct HttpRequest {
    std::string method;
    std::string path;
    std::string query;
    std::string body;
};

std::string httpResponse(const std::string& status, const std::string& contentType, const std::string& body, const std::string& extra = "") {
    std::ostringstream output;
    output << "HTTP/1.1 " << status << "\r\n"
           << "Content-Type: " << contentType << "\r\n"
           << "Content-Length: " << body.size() << "\r\n"
           << "Cache-Control: no-store\r\n"
           << extra
           << "Connection: close\r\n\r\n"
           << body;
    return output.str();
}

void sendAll(int client, const std::string& data) {
    size_t sent = 0;
    while (sent < data.size()) {
        const ssize_t n = send(client, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
        if (n <= 0) return;
        sent += static_cast<size_t>(n);
    }
}

bool readRequest(int client, HttpRequest& request) {
    timeval timeout{5, 0};
    setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    std::string raw;
    char buffer[4096];
    size_t headerEnd = std::string::npos;
    while (headerEnd == std::string::npos) {
        const ssize_t n = recv(client, buffer, sizeof(buffer), 0);
        if (n <= 0) return false;
        raw.append(buffer, static_cast<size_t>(n));
        headerEnd = raw.find("\r\n\r\n");
        if (raw.size() > 65536) return false;
    }
    const std::string head = raw.substr(0, headerEnd);
    std::istringstream line(head.substr(0, head.find("\r\n")));
    std::string target;
    line >> request.method >> target;
    const auto q = target.find('?');
    request.path = target.substr(0, q);
    request.query = q == std::string::npos ? "" : target.substr(q + 1);

    size_t contentLength = 0;
    std::string lowerHead = head;
    std::transform(lowerHead.begin(), lowerHead.end(), lowerHead.begin(), [](unsigned char c) { return std::tolower(c); });
    const auto cl = lowerHead.find("content-length:");
    if (cl != std::string::npos) contentLength = std::strtoul(head.c_str() + cl + 15, nullptr, 10);
    if (contentLength > 65536) return false;
    request.body = raw.substr(headerEnd + 4);
    while (request.body.size() < contentLength) {
        const ssize_t n = recv(client, buffer, sizeof(buffer), 0);
        if (n <= 0) break;
        request.body.append(buffer, static_cast<size_t>(n));
    }
    return true;
}

std::string queryParam(const std::string& query, const std::string& key) {
    std::istringstream stream(query);
    std::string pair;
    while (std::getline(stream, pair, '&')) {
        const auto eq = pair.find('=');
        if (pair.substr(0, eq) == key) return eq == std::string::npos ? "" : pair.substr(eq + 1);
    }
    return "";
}

std::string readingJson(const Reading& r) {
    return "{\"timestamp\":\"" + r.timestamp + "\",\"antenna\":" + std::to_string(r.antenna) + ",\"epc\":\"" + jsonEscape(r.epc) +
           "\",\"rssi\":" + std::to_string(r.rssi) + ",\"rssi_max\":" + std::to_string(r.rssiMax) + ",\"count\":" + std::to_string(r.count) + "}";
}

std::string readingsJson(const std::vector<Reading>& list, int filterAntenna) {
    std::string body = "{\"readings\":[";
    bool first = true;
    for (const auto& r : list) {
        if (filterAntenna != 0 && r.antenna != filterAntenna) continue;
        if (!first) body += ",";
        first = false;
        body += readingJson(r);
    }
    return body + "]}";
}

std::string stateJson() {
    std::ostringstream out;
    out << "{\"reader\":" << (readerConnected ? "true" : "false")
        << ",\"reader_type\":" << readerTypeCode.load()
        << ",\"esp32\":" << (esp32Connected ? "true" : "false")
        << ",\"scanning\":" << (scanEnabled ? "true" : "false")
        << ",\"mode\":\"" << (bufferMode ? "buffer" : "normal") << "\""
        << ",\"buffer_log_available\":" << (bufferLogAvailable ? "true" : "false");
    {
        const Config cfg = configSnapshot();
        out << ",\"gate_mode\":" << cfg.gateMode << ",\"gate\":" << effectiveGate(cfg.gateMode) << ",\"esp32_gate\":" << esp32Gate.load();
    }
    std::lock_guard<std::mutex> lock(antennaMutex);
    out << ",\"tags\":" << liveTags.size() << ",\"log_rows\":" << readings.size() << ",\"buffer_rows\":" << bufferReadings.size() << ",\"antennas\":[";
    for (int i = 0; i < ReceiverCount; ++i)
        out << (i ? "," : "") << "{\"id\":" << i + 1 << ",\"detected\":" << (antennaStates[i].detected ? "true" : "false")
            << ",\"applied_power\":" << appliedPower[i].load() << "}";
    out << "]}";
    return out.str();
}

std::string liveJson(int windowMs) {
    const int64_t nowMs = epochMillisNow();
    std::ostringstream out;
    out << std::fixed << std::setprecision(1);
    out << "{\"now\":" << nowMs << ",\"tags\":[";
    bool first = true;
    std::lock_guard<std::mutex> lock(antennaMutex);
    for (const auto& kv : liveTags) {
        if (nowMs - kv.second.lastMs > windowMs) continue;
        if (!first) out << ",";
        first = false;
        out << "{\"epc\":\"" << kv.first << "\",\"age\":" << nowMs - kv.second.lastMs << ",\"ant\":[";
        for (int i = 0; i < ReceiverCount; ++i) {
            const auto& a = kv.second.ant[i];
            if (i) out << ",";
            if (a.lastMs == 0 || nowMs - a.lastMs > windowMs) out << "null";
            else out << "{\"rssi\":" << a.rssi << ",\"avg\":" << a.avg << ",\"age\":" << nowMs - a.lastMs << ",\"first\":" << nowMs - a.firstMs << ",\"hits\":" << a.hits << "}";
        }
        out << "]}";
    }
    out << "]}";
    return out.str();
}

std::string csvExport(const std::vector<Reading>& list) {
    std::string csv = "timestamp,antenna,epc,rssi,rssi_max,count\n";
    for (const auto& r : list)
        csv += r.timestamp + "," + std::to_string(r.antenna) + "," + r.epc + "," + std::to_string(r.rssi) + "," + std::to_string(r.rssiMax) + "," + std::to_string(r.count) + "\n";
    return csv;
}

std::string runTagJob(const std::shared_ptr<TagJob>& job, int& httpStatus) {
    if (!readerConnected) {
        httpStatus = 503;
        return "{\"ok\":false,\"message\":\"reader not connected\"}";
    }
    auto future = job->promise.get_future();
    {
        std::lock_guard<std::mutex> lock(jobMutex);
        if (pendingJob) {
            httpStatus = 409;
            return "{\"ok\":false,\"message\":\"another tag job is running\"}";
        }
        pendingJob = job;
    }
    if (future.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
        httpStatus = 504;
        return "{\"ok\":false,\"message\":\"reader did not finish in time\"}";
    }
    const TagJobResult result = future.get();
    httpStatus = 200;
    std::ostringstream out;
    out << "{\"ok\":" << (result.ok ? "true" : "false") << ",\"message\":\"" << jsonEscape(result.message) << "\"";
    if (!result.readBack.empty()) out << ",\"read_back\":\"" << result.readBack << "\"";
    out << ",\"seen\":[";
    for (size_t i = 0; i < result.seen.size(); ++i)
        out << (i ? "," : "") << "{\"epc\":\"" << result.seen[i].epc << "\",\"rssi\":" << result.seen[i].rssi << "}";
    out << "]}";
    return out.str();
}

std::string contentTypeFor(const std::string& path) {
    const auto dot = path.rfind('.');
    const std::string ext = dot == std::string::npos ? "" : path.substr(dot + 1);
    if (ext == "png") return "image/png";
    if (ext == "gif") return "image/gif";
    if (ext == "jpg" || ext == "jpeg") return "image/jpeg";
    if (ext == "svg") return "image/svg+xml";
    if (ext == "js") return "application/javascript";
    if (ext == "css") return "text/css";
    if (ext == "json") return "application/json";
    return "application/octet-stream";
}

void handleClient(int client) {
    HttpRequest req;
    if (!readRequest(client, req)) {
        close(client);
        return;
    }
    const std::string json = "application/json";
    std::string out;
    auto ok = [&](const std::string& body) { out = httpResponse("200 OK", json, body); };
    auto bad = [&](const std::string& message) { out = httpResponse("400 Bad Request", json, "{\"status\":\"error\",\"message\":\"" + jsonEscape(message) + "\"}"); };
    const std::string& m = req.method;
    const std::string& p = req.path;

    if (m == "GET" && p == "/api/state") {
        ok(stateJson());
    } else if (m == "GET" && p == "/api/live") {
        int window = std::atoi(queryParam(req.query, "window").c_str());
        if (window <= 0) window = 3000;
        ok(liveJson(std::min(window, 120000)));
    } else if (m == "GET" && p == "/api/config") {
        ok(configToJson(configSnapshot()));
    } else if (m == "POST" && p == "/api/config") {
        std::lock_guard<std::mutex> lock(configMutex);
        Config updated = config;
        std::string error;
        if (!applyConfigJson(updated, req.body, error)) {
            bad(error);
        } else {
            config = updated;
            const bool saved = saveConfigLocked();
            ok("{\"status\":\"success\",\"saved\":" + std::string(saved ? "true" : "false") + ",\"config\":" + configToJson(config) + "}");
        }
    } else if (m == "POST" && p == "/api/antenna-power") {
        // Legacy: {"power":N} sets every antenna; {"antenna":A,"power":N} sets one.
        const int power = jsonInteger(req.body, "power");
        const int antenna = jsonInteger(req.body, "antenna", 0);
        if (power < 0 || power > 33 || antenna < 0 || antenna > ReceiverCount) {
            bad("power must be 0-33 dBm");
        } else {
            std::lock_guard<std::mutex> lock(configMutex);
            for (int i = 0; i < ReceiverCount; ++i)
                if (antenna == 0 || antenna == i + 1) config.power[i] = power;
            saveConfigLocked();
            ok("{\"status\":\"success\",\"message\":\"RF power saved\",\"power\":" + std::to_string(power) + "}");
        }
    } else if (m == "POST" && (p == "/api/tag/probe" || p == "/api/tag/write")) {
        auto job = std::make_shared<TagJob>();
        job->kind = p == "/api/tag/probe" ? "probe" : "write";
        job->antenna = jsonInteger(req.body, "antenna", 1);
        job->power = jsonInteger(req.body, "power", -1);
        job->target = jsonValue(req.body, "target");
        job->mem = jsonValue(req.body, "mem");
        if (job->mem != "user") job->mem = "epc";
        job->wordPtr = jsonInteger(req.body, "word_ptr", 0);
        job->data = jsonValue(req.body, "data");
        job->password = jsonValue(req.body, "password");
        if (job->antenna < 1 || job->antenna > ReceiverCount) {
            bad("antenna must be 1-4");
        } else if (job->power > 33) {
            bad("power must be 0-33 dBm");
        } else {
            int status = 200;
            const std::string body = runTagJob(job, status);
            out = httpResponse(status == 200 ? "200 OK" : std::to_string(status) + " Error", json, body);
        }
    } else if (m == "GET" && p == "/api/get-ip") {
        ok("{\"ip\":\"" + localIp() + "\",\"gateway\":\"" + defaultGateway() + "\",\"status\":\"ok\"}");
    } else if (m == "GET" && p == "/api/reader-info") {
        ok("{\"receiver_count\":" + std::to_string(ReceiverCount) + "}");
    } else if (m == "GET" && p == "/api/status-log") {
        const int filter = std::atoi(queryParam(req.query, "antenna").c_str());
        std::lock_guard<std::mutex> lock(antennaMutex);
        ok(readingsJson(readings, filter));
    } else if (m == "GET" && p == "/api/buffer-log") {
        std::lock_guard<std::mutex> lock(antennaMutex);
        ok(readingsJson(bufferReadings, 0));
    } else if (m == "GET" && p == "/api/export.csv") {
        std::string csv;
        {
            std::lock_guard<std::mutex> lock(antennaMutex);
            csv = csvExport(queryParam(req.query, "source") == "buffer" ? bufferReadings : readings);
        }
        out = httpResponse("200 OK", "text/csv; charset=utf-8", csv, "Content-Disposition: attachment; filename=\"rfid-" + std::to_string(epochMillisNow() / 1000) + ".csv\"\r\n");
    } else if (m == "POST" && p == "/api/status-clear") {
        std::lock_guard<std::mutex> lock(antennaMutex);
        readings.clear();
        liveTags.clear();
        ok("{\"status\":\"success\",\"message\":\"Status log cleared\"}");
    } else if (m == "POST" && p == "/api/buffer-clear") {
        std::lock_guard<std::mutex> lock(antennaMutex);
        bufferReadings.clear();
        bufferLogAvailable = false;
        ok("{\"status\":\"success\",\"message\":\"Buffer log cleared\"}");
    } else if (m == "GET" && p == "/api/antenna-status") {
        std::lock_guard<std::mutex> lock(antennaMutex);
        std::string body = "{\"source\":\"" + std::string(readerConnected ? "reader" : "unavailable") + "\",\"antennas\":[";
        for (int i = 0; i < ReceiverCount; ++i)
            body += std::string(i ? "," : "") + "{\"id\":" + std::to_string(i + 1) + ",\"detected\":" + (antennaStates[i].detected ? "true" : "false") + "}";
        ok(body + "]}");
    } else if (m == "GET" && p == "/api/scan-status") {
        ok("{\"scanning\":" + std::string(scanEnabled ? "true" : "false") + ",\"mode\":\"" + (bufferMode ? "buffer" : "normal") + "\",\"buffer_log_available\":" + (bufferLogAvailable ? "true" : "false") + "}");
    } else if (m == "POST" && p == "/api/scan-control") {
        if (applyScanControl(jsonValue(req.body, "action"), jsonValue(req.body, "mode")))
            ok("{\"status\":\"success\",\"scanning\":" + std::string(scanEnabled ? "true" : "false") + ",\"mode\":\"" + (bufferMode ? "buffer" : "normal") + "\"}");
        else
            bad("action must be start or stop");
    } else if (m == "POST" && p == "/api/update_network") {
        if (updateNetwork(jsonValue(req.body, "ip"), jsonValue(req.body, "gateway")))
            ok("{\"status\":\"success\",\"message\":\"Network updated\"}");
        else
            bad("Invalid network data or netplan failed");
    } else if (m == "GET" && p.rfind("/assets/", 0) == 0 && p.find("..") == std::string::npos) {
        const std::string file = readFile(WebRoot + p);
        out = file.empty() ? httpResponse("404 Not Found", "text/plain", "Not found") : httpResponse("200 OK", contentTypeFor(p), file);
    } else if (m == "GET" && (p == "/" || p == "/index.html")) {
        out = httpResponse("200 OK", "text/html; charset=utf-8", readFile(WebRoot + "/index.html"));
    } else {
        out = httpResponse("404 Not Found", "text/plain; charset=utf-8", "Not found");
    }
    sendAll(client, out);
    close(client);
}
}  // namespace

int main() {
    std::signal(SIGPIPE, SIG_IGN);

    if (gpioInit()) {
        gpioSetOutput(PiReadyLedPin);
        gpioSetOutput(PiEsp32LinkLedPin);
        gpioWrite(PiReadyLedPin, true);
    } else {
        std::cerr << "GPIO: /dev/gpiomem unavailable, panel LEDs disabled" << std::endl;
    }
    for (auto& p : appliedPower) p = -1;
    loadConfig();

    const int server = socket(AF_INET, SOCK_STREAM, 0);
    if (server < 0) return 1;
    int reuse = 1;
    setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(static_cast<uint16_t>(HttpPort));
    if (bind(server, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0 || listen(server, 32) < 0) {
        std::cerr << "HTTP: cannot listen on port " << HttpPort << std::endl;
        close(server);
        return 1;
    }

    std::cout << "rfid-web-cpp listening on port " << HttpPort << ", web root " << WebRoot << std::endl;
    std::thread reader(readerLoop);
    std::thread esp32(esp32Loop);
    while (true) {
        const int client = accept(server, nullptr, nullptr);
        if (client >= 0) std::thread(handleClient, client).detach();
    }
    readerRunning = false;
    esp32BridgeRunning = false;
    reader.join();
    esp32.join();
}
