// ESP32 (LilyGo TTGO T-Display) firmware.
//
// Receives newline-delimited JSON frames from a Raspberry Pi over Bluetooth
// Classic SPP and renders the latest RFID reading for two antennas at a
// time on the built-in display.
//
// - GPIO25 (digital input, interrupt-driven) selects which channel is
//   displayed: LOW = channel 1 (antennas 1-2), HIGH = channel 2
//   (antennas 3-4). This tracks external hardware (e.g. the antenna bank
//   switch), it is not a manual UI toggle.
// - Every distinct EPC seen is kept in an export table and counted on
//   screen ("Logged: N").
// - GPIO0 (onboard top button) packages that table as CSV and sends it
//   over the active Bluetooth connection, for a PC that has connected in
//   place of the Pi to receive it (see ../tools/pc_bluetooth_csv_receiver.py).
//
// Wire format is documented in ../PROTOCOL.md.

#include <Arduino.h>
#include "BluetoothSerial.h"
#include "esp_bt_device.h"
#include <ArduinoJson.h>
#include <TFT_eSPI.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled. Use an ESP32 build with Bluetooth Classic (Bluedroid) support.
#endif

namespace {

constexpr const char *kBtDeviceName = "ESP32-RFID-Display";
constexpr uint8_t kChannelPin = 25;      // digital 0/1 channel-select input, interrupt-driven
constexpr uint8_t kExportButtonPin = 0;  // onboard top button: package + send the log

constexpr size_t kLineBufferSize = 512;
constexpr uint32_t kDebounceMs = 250;
constexpr uint32_t kChannelPinDebounceMs = 30;
constexpr uint32_t kStaleTagMs = 5000;
constexpr uint32_t kPeriodicRedrawMs = 500;
constexpr uint32_t kExportMessageMs = 3000;
constexpr size_t kMaxExportEntries = 512;

BluetoothSerial gSerialBt;
TFT_eSPI gTft;
TFT_eSprite gScreen(&gTft);

char gLineBuffer[kLineBufferSize];
size_t gLineLength = 0;

struct AntennaState {
  bool hasTag = false;
  String epc;
  int rssi = 0;
  uint32_t count = 0;
  uint32_t lastSeenMs = 0;
};

// Index 0 unused; antennas are numbered 1-4 to match the physical ports.
// Holds only the latest reading per antenna, for the live display cards.
AntennaState gAntennas[5];

// One row per distinct (antenna, EPC) seen since the last export. This is
// what "Logged: N" counts and what the export button sends as CSV.
struct ExportEntry {
  uint8_t channel;
  uint8_t antenna;
  char epc[25];
  int16_t rssi;
  uint32_t count;
  uint32_t lastSeenMs;
};

ExportEntry gExportLog[kMaxExportEntries];
size_t gExportLogSize = 0;

struct DeviceStatus {
  String deviceId = "--";
  uint8_t scanState = 0; // 0 stopped, 1 running, 2 complete, 3 error
  String errorText;
  bool hasError = false;
} gStatus;

volatile bool gBtConnected = false;
volatile bool gChannelPinDirty = true; // force an initial read at boot
uint8_t gCurrentChannel = 1;           // 1 or 2, which antenna pair is displayed

bool gScreenDirty = true;
uint32_t gLastExportButtonMs = 0;
String gExportMessage;
uint32_t gExportMessageUntilMs = 0;

uint8_t channelForAntenna(uint8_t antenna) { return antenna <= 2 ? 1 : 2; }

void IRAM_ATTR channelPinIsr() { gChannelPinDirty = true; }

void btEventCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t * /*param*/) {
  if (event == ESP_SPP_SRV_OPEN_EVT || event == ESP_SPP_OPEN_EVT) {
    gBtConnected = true;
    gScreenDirty = true;
    Serial.println("[BT] client connected");
  } else if (event == ESP_SPP_CLOSE_EVT) {
    gBtConnected = false;
    gScreenDirty = true;
    Serial.println("[BT] client disconnected");
  }
}

void sendJsonLine(JsonDocument &doc) {
  String out;
  serializeJson(doc, out);
  out += '\n';
  gSerialBt.print(out);
}

// Updates (or inserts) the export-table row for one antenna/EPC pair.
void recordExportEntry(uint8_t antenna, const char *epc, int rssi, uint32_t count) {
  uint8_t channel = channelForAntenna(antenna);
  for (size_t i = 0; i < gExportLogSize; ++i) {
    ExportEntry &entry = gExportLog[i];
    if (entry.antenna == antenna && strcmp(entry.epc, epc) == 0) {
      entry.rssi = static_cast<int16_t>(rssi);
      entry.count = count;
      entry.lastSeenMs = millis();
      return;
    }
  }
  if (gExportLogSize >= kMaxExportEntries) return; // table full: keep existing rows fresh, drop new tags

  ExportEntry &entry = gExportLog[gExportLogSize++];
  entry.channel = channel;
  entry.antenna = antenna;
  strncpy(entry.epc, epc, sizeof(entry.epc) - 1);
  entry.epc[sizeof(entry.epc) - 1] = '\0';
  entry.rssi = static_cast<int16_t>(rssi);
  entry.count = count;
  entry.lastSeenMs = millis();
}

void handleScanResult(JsonDocument &doc) {
  int antenna = doc["antenna"] | 0;
  if (antenna < 1 || antenna > 4) return;

  const char *epc = doc["epc"] | "";
  int rssi = doc["rssi"] | 0;
  uint32_t count = doc["count"] | 1;

  AntennaState &state = gAntennas[antenna];
  state.hasTag = true;
  state.epc = epc;
  state.rssi = rssi;
  state.count = count;
  state.lastSeenMs = millis();

  recordExportEntry(static_cast<uint8_t>(antenna), epc, rssi, count);

  if (channelForAntenna(static_cast<uint8_t>(antenna)) == gCurrentChannel) {
    gScreenDirty = true;
  }
}

void handleStatus(JsonDocument &doc) {
  gStatus.deviceId = String((const char *)(doc["device_id"] | gStatus.deviceId.c_str()));
  gStatus.scanState = doc["scan_state"] | gStatus.scanState;
  const char *errorText = doc["error_text"] | "";
  gStatus.hasError = gStatus.scanState == 3 && strlen(errorText) > 0;
  gStatus.errorText = errorText;
  gScreenDirty = true;
}

void handleError(JsonDocument &doc) {
  gStatus.hasError = true;
  gStatus.errorText = String((const char *)(doc["message"] | "error"));
  gScreenDirty = true;
}

void handlePing() {
  JsonDocument reply;
  reply["type"] = "pong";
  reply["ts"] = static_cast<uint32_t>(millis());
  sendJsonLine(reply);
}

void processLine(char *line, size_t length) {
  if (length == 0) return;

  JsonDocument doc;
  if (deserializeJson(doc, line, length) != DeserializationError::Ok) {
    Serial.printf("[BT] dropped malformed line (%u bytes)\n", static_cast<unsigned>(length));
    return; // malformed frame: drop it and resync on the next newline
  }

  const char *type = doc["type"] | "";
  if (strcmp(type, "scan_result") == 0) {
    handleScanResult(doc);
  } else if (strcmp(type, "status") == 0 || strcmp(type, "hello") == 0) {
    handleStatus(doc);
  } else if (strcmp(type, "ping") == 0) {
    handlePing();
  } else if (strcmp(type, "error") == 0) {
    handleError(doc);
  }
  // Unknown types are ignored so the schema can grow later.
}

void pollBluetooth() {
  while (gSerialBt.available()) {
    char c = static_cast<char>(gSerialBt.read());
    if (c == '\n') {
      processLine(gLineBuffer, gLineLength);
      gLineLength = 0;
      continue;
    }
    if (c == '\r') continue;
    if (gLineLength < kLineBufferSize - 1) {
      gLineBuffer[gLineLength++] = c;
    } else {
      gLineLength = 0; // line too long: drop and resync
    }
  }
}

// GPIO25 reflects an external channel-select signal (e.g. the antenna bank
// switch), not a manual UI control: LOW = channel 1, HIGH = channel 2.
void pollChannelPin() {
  static uint32_t lastChangeMs = 0;
  if (!gChannelPinDirty) return;

  uint32_t now = millis();
  if (now - lastChangeMs < kChannelPinDebounceMs) return;
  gChannelPinDirty = false;
  lastChangeMs = now;

  uint8_t newChannel = (digitalRead(kChannelPin) == LOW) ? 1 : 2;
  if (newChannel != gCurrentChannel) {
    gCurrentChannel = newChannel;
    gScreenDirty = true;
    Serial.printf("[CH] switched to channel %u\n", newChannel);
  }
}

void exportLogOverBluetooth() {
  gSerialBt.print("CSV_EXPORT_BEGIN\n");
  gSerialBt.print("channel,antenna,epc,rssi_dbm,count,age_s\n");

  uint32_t now = millis();
  for (size_t i = 0; i < gExportLogSize; ++i) {
    const ExportEntry &entry = gExportLog[i];
    uint32_t ageS = (now - entry.lastSeenMs) / 1000;
    gSerialBt.printf("%u,%u,%s,%d,%u,%u\n", entry.channel, entry.antenna, entry.epc, entry.rssi, entry.count, ageS);
  }

  gSerialBt.print("CSV_EXPORT_END\n");
}

// Onboard top button: package the export table as CSV and send it over
// whatever is currently connected via Bluetooth, then start a fresh batch.
void pollExportButton() {
  uint32_t now = millis();
  if (now - gLastExportButtonMs < kDebounceMs) return;
  if (digitalRead(kExportButtonPin) != LOW) return;

  gLastExportButtonMs = now;
  if (gBtConnected) {
    size_t exported = gExportLogSize;
    exportLogOverBluetooth();
    gExportLogSize = 0; // next button press starts a new batch
    gExportMessage = "Exported " + String(exported) + " rows";
    Serial.printf("[EXPORT] sent %u rows\n", static_cast<unsigned>(exported));
  } else {
    gExportMessage = "No BT link - cannot export";
    Serial.println("[EXPORT] button pressed but no BT link");
  }
  gExportMessageUntilMs = now + kExportMessageMs;
  gScreenDirty = true;
}

const char *scanStateText(uint8_t state) {
  switch (state) {
    case 1: return "RUNNING";
    case 2: return "COMPLETE";
    case 3: return "ERROR";
    default: return "STOPPED";
  }
}

void drawAntennaCard(int x, int y, int w, int h, uint8_t antennaNumber) {
  const AntennaState &state = gAntennas[antennaNumber];

  gScreen.drawRoundRect(x, y, w, h, 4, TFT_DARKGREY);
  gScreen.setTextDatum(TL_DATUM);
  gScreen.setTextColor(TFT_CYAN, TFT_BLACK);
  gScreen.drawString("ANT " + String(antennaNumber), x + 6, y + 4, 2);

  if (!state.hasTag) {
    gScreen.setTextColor(TFT_DARKGREY, TFT_BLACK);
    gScreen.drawString("no tag", x + 6, y + 28, 2);
    return;
  }

  uint32_t ageMs = millis() - state.lastSeenMs;
  bool stale = ageMs > kStaleTagMs;
  gScreen.setTextColor(stale ? TFT_DARKGREY : TFT_WHITE, TFT_BLACK);

  String epcShort = state.epc.length() > 14 ? state.epc.substring(state.epc.length() - 14) : state.epc;
  gScreen.drawString(epcShort, x + 6, y + 26, 2);
  gScreen.drawString("RSSI " + String(state.rssi) + "dBm", x + 6, y + 46, 1);
  gScreen.drawString("x" + String(state.count) + "  " + String(ageMs / 1000) + "s ago", x + 6, y + 58, 1);
}

// The single dynamic footer line, in priority order: a transient export
// result/error, then a reader error, otherwise the running logged count.
void drawFooterStatusLine(int x, int y) {
  gScreen.setTextDatum(TL_DATUM);
  if (millis() < gExportMessageUntilMs) {
    gScreen.setTextColor(TFT_YELLOW, TFT_BLACK);
    gScreen.drawString(gExportMessage, x, y, 1);
  } else if (gStatus.hasError) {
    gScreen.setTextColor(TFT_RED, TFT_BLACK);
    gScreen.drawString(gStatus.errorText, x, y, 1);
  } else {
    gScreen.setTextColor(TFT_SILVER, TFT_BLACK);
    gScreen.drawString("Logged: " + String(gExportLogSize), x, y, 1);
  }
}

void redraw() {
  gScreen.fillSprite(TFT_BLACK);

  gScreen.setTextDatum(TL_DATUM);
  gScreen.setTextColor(TFT_WHITE, TFT_BLACK);
  gScreen.drawString("RFID Monitor", 4, 2, 2);

  gScreen.fillCircle(232, 8, 4, gBtConnected ? TFT_GREEN : TFT_RED);

  gScreen.setTextDatum(TR_DATUM);
  gScreen.setTextColor(TFT_YELLOW, TFT_BLACK);
  gScreen.drawString("CH " + String(gCurrentChannel), 220, 2, 2);

  uint8_t firstAntenna = (gCurrentChannel == 1) ? 1 : 3;
  drawAntennaCard(4, 22, 115, 78, firstAntenna);
  drawAntennaCard(121, 22, 115, 78, firstAntenna + 1);

  gScreen.setTextDatum(TL_DATUM);
  gScreen.setTextColor(TFT_SILVER, TFT_BLACK);
  gScreen.drawString(gStatus.deviceId + " - " + scanStateText(gStatus.scanState), 4, 102, 1);

  drawFooterStatusLine(4, 113);

  gScreen.setTextColor(gBtConnected ? TFT_GREEN : TFT_RED, TFT_BLACK);
  gScreen.drawString(gBtConnected ? "BT connected" : "BT waiting...", 4, 124, 1);

  gScreen.pushSprite(0, 0);
}

} // namespace

void setup() {
  Serial.begin(115200);

  pinMode(kExportButtonPin, INPUT_PULLUP);

  pinMode(kChannelPin, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(kChannelPin), channelPinIsr, CHANGE);
  gCurrentChannel = (digitalRead(kChannelPin) == LOW) ? 1 : 2;

  gTft.init();
  gTft.setRotation(1); // landscape, 240x135
  gTft.fillScreen(TFT_BLACK);
  gScreen.createSprite(240, 135);
  gScreen.setSwapBytes(true);

  gSerialBt.register_callback(btEventCallback);
  gSerialBt.begin(kBtDeviceName);

  const uint8_t *btMac = esp_bt_dev_get_address();
  if (btMac != nullptr) {
    Serial.printf("[BT] MAC address: %02X:%02X:%02X:%02X:%02X:%02X\n", btMac[0], btMac[1], btMac[2], btMac[3],
                  btMac[4], btMac[5]);
  }

  gScreenDirty = true;
}

void loop() {
  pollBluetooth();
  pollChannelPin();
  pollExportButton();

  static uint32_t lastRedraw = 0;
  uint32_t now = millis();
  if (gScreenDirty || now - lastRedraw > kPeriodicRedrawMs) {
    redraw();
    gScreenDirty = false;
    lastRedraw = now;
  }
}
