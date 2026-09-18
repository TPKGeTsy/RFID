// ESP32 (LilyGo TTGO T-Display) firmware.
//
// Talks newline-delimited JSON to the Pi over a wired RS-422 UART
// (GPIO39 RX / GPIO13 TX) - see PROTOCOL.md for the message types
// (status/scan_result/scan_control/ping/pong/error). The display always
// shows the latest reading for one of 2 channels at a time - each
// channel combines a pair of antennas in hardware before the Pi ever
// sees the data, so there is one reading per channel, not per antenna.
//
// - GPIO25 (digital input, interrupt-driven) selects which channel is
//   displayed: LOW = channel 1, HIGH = channel 2. This tracks external
//   hardware (e.g. the antenna bank switch), it is not a manual UI
//   toggle - but every change is also sent to the Pi as a `select`
//   command.
// - Every distinct EPC seen is kept in an export table, counted overall
//   ("Logged: N") and per channel (shown on that channel's card).
// - Panel buttons (GPIO0/27 white, GPIO26 yellow, GPIO32 green, GPIO33
//   red) each send a `scan_control` command to the Pi: select (the
//   channel switch, above), start, stop, reset, export - see
//   PROTOCOL.md. Reset also clears the ESP32's own tag tables; export
//   also types the current table into a BLE HID keyboard link
//   ("ESP32-RFID") a PC can pair with, then puts the ESP32 into a live
//   mode where every new tag arriving from the Pi is typed as it comes
//   in, until stop is pressed.
// - GPIO15/22/17/21 are opto-isolated outputs that directly mirror the
//   yellow/white/green/red button inputs while they are held (steady
//   level, not a pulse) - a hardware echo for external equipment,
//   independent of what those buttons also trigger above.
// - GPIO35/36/2/12 (PLACEHOLDERS - confirm against the real board before
//   wiring; several assumed-free pins on this project have turned out to
//   already be spoken for) read feedback back from those same four
//   optocouplers. Captured into state but not yet wired to any output -
//   what to do with it is still unspecified.
//
// Wire format is documented in ../PROTOCOL.md.

#include <Arduino.h>
#include <ArduinoJson.h>
#include <BleKeyboard.h>
#include <TFT_eSPI.h>

namespace {

constexpr const char *kBleKeyboardName = "RFID-Scanner";
constexpr uint8_t kChannelPin = 25;       // digital 0/1 channel-select input, interrupt-driven
constexpr uint8_t kExportButtonPin = 0;   // onboard top button: package + send the log
constexpr uint8_t kWhiteButtonPin = 27;   // external panel button: same as kExportButtonPin
constexpr uint8_t kYellowButtonPin = 26;  // external panel button: hold to clear the log
constexpr uint8_t kGreenButtonPin = 32;   // external panel button: tell the Pi to start scanning
constexpr uint8_t kRedButtonPin = 33;     // external panel button: tell the Pi to stop scanning

// Opto-isolated outputs, one per panel button, mirroring its pressed state.
constexpr uint8_t kYellowEchoPin = 15;
constexpr uint8_t kWhiteEchoPin = 22;
constexpr uint8_t kGreenEchoPin = 17;
constexpr uint8_t kRedEchoPin = 21;

// Feedback read back from those same four optocouplers. PLACEHOLDER pin
// numbers - confirm against the real board before wiring; several other
// "free" pins on this project turned out to already be spoken for.
constexpr uint8_t kYellowFeedbackPin = 35;
constexpr uint8_t kWhiteFeedbackPin = 36;
constexpr uint8_t kGreenFeedbackPin = 2;
constexpr uint8_t kRedFeedbackPin = 12;

// Wired RS-422 link to the Pi: newline-delimited JSON, both directions.
// RS-422 module terminal wiring: A+ -> kWireRxPin, A- -> kWireTxPin.
constexpr uint8_t kWireRxPin = 39;
constexpr uint8_t kWireTxPin = 13;
constexpr uint32_t kWireBaud = 57600;

constexpr size_t kLineBufferSize = 512;
constexpr uint32_t kDebounceMs = 250;
constexpr uint32_t kChannelPinDebounceMs = 30;
constexpr uint32_t kStaleTagMs = 5000;
constexpr uint32_t kPeriodicRedrawMs = 500;
constexpr uint32_t kStatusMessageMs = 3000;
constexpr uint32_t kClearHoldMs = 1500;
constexpr size_t kMaxExportEntries = 512;

BleKeyboard gBleKeyboard(kBleKeyboardName, "LilyGo", 100);
HardwareSerial gWireSerial(2);
TFT_eSPI gTft;
TFT_eSprite gScreen(&gTft);

char gWireLineBuffer[kLineBufferSize];
size_t gWireLineLength = 0;

// While a BLE-keyboard typing call is in progress (single live row or a
// full-table export), the wire UART is still drained inside its delay loop
// so bytes from the Pi are never lost - but any tag that arrives mid-type
// is queued here instead of typing it right away, which would otherwise
// recurse back into the typing code from inside its own delay.
volatile bool gTypingInProgress = false;
String gPendingLiveRows;

struct ChannelState {
  bool hasTag = false;
  String epc;
  int rssi = 0;
  uint32_t count = 0;
  uint32_t lastSeenMs = 0;
};

// Index 0 unused; channels are numbered 1-2. Holds only the latest
// combined reading per channel, for the live display card.
ChannelState gChannels[3];

// One row per distinct (channel, EPC) seen since the last export. This is
// what "Logged: N" counts and what the export button sends as CSV/typing.
struct ExportEntry {
  uint8_t channel;
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

volatile bool gChannelPinDirty = true; // force an initial read at boot
uint8_t gCurrentChannel = 1;           // 1 or 2, which antenna pair is displayed

bool gScreenDirty = true;
uint32_t gLastExportButtonMs = 0;
uint32_t gLastControlButtonMs = 0;
uint32_t gYellowPressStartMs = 0;
bool gYellowHoldHandled = false;
String gStatusMessage;
uint32_t gStatusMessageUntilMs = 0;

// True from an "export" press until the next "stop": while active, every
// new tag reported by the Pi is typed into the BLE keyboard live, one by
// one, instead of waiting for the next export button press.
bool gLiveExportActive = false;

// Feedback read back from the four optocoupler outputs. Captured each
// loop; not yet wired to any display or outbound message.
bool gOptoFeedback[4] = {false, false, false, false};

uint8_t channelForAntenna(uint8_t antenna) { return antenna <= 2 ? 1 : 2; }

void IRAM_ATTR channelPinIsr() { gChannelPinDirty = true; }

void sendJsonToPi(JsonDocument &doc) {
  String out;
  serializeJson(doc, out);
  out += '\n';
  gWireSerial.print(out);
}

// Updates (or inserts) the export-table row for one channel/EPC pair.
void recordExportEntry(uint8_t channel, const char *epc, int rssi, uint32_t count) {
  for (size_t i = 0; i < gExportLogSize; ++i) {
    ExportEntry &entry = gExportLog[i];
    if (entry.channel == channel && strcmp(entry.epc, epc) == 0) {
      entry.rssi = static_cast<int16_t>(rssi);
      entry.count = count;
      entry.lastSeenMs = millis();
      return;
    }
  }
  if (gExportLogSize >= kMaxExportEntries) return; // table full: keep existing rows fresh, drop new tags

  ExportEntry &entry = gExportLog[gExportLogSize++];
  entry.channel = channel;
  strncpy(entry.epc, epc, sizeof(entry.epc) - 1);
  entry.epc[sizeof(entry.epc) - 1] = '\0';
  entry.rssi = static_cast<int16_t>(rssi);
  entry.count = count;
  entry.lastSeenMs = millis();
}

size_t countTagsForChannel(uint8_t channel) {
  size_t count = 0;
  for (size_t i = 0; i < gExportLogSize; ++i) {
    if (gExportLog[i].channel == channel) ++count;
  }
  return count;
}

void typeCharByChar(const String &text); // defined below, used by the live-export path
void flushPendingLiveRows(); // defined below, used by the live-export path

// Applies one tag reading reported by the Pi. The antenna's two-per-
// channel pairing is merged upstream (in hardware, before the Pi), so
// "antenna" here only tells us which channel it belongs to; it is not a
// distinct display slot (see the file header comment).
void applyTagReading(uint8_t antenna, const char *epc, int rssi, uint32_t count) {
  if (antenna < 1 || antenna > 4) return;
  uint8_t channel = channelForAntenna(antenna);

  ChannelState &state = gChannels[channel];
  state.hasTag = true;
  state.epc = epc;
  state.rssi = rssi;
  state.count = count;
  state.lastSeenMs = millis();

  recordExportEntry(channel, epc, rssi, count);

  if (channel == gCurrentChannel) {
    gScreenDirty = true;
  }

  if (gLiveExportActive && gBleKeyboard.isConnected()) {
    String row = String(channel) + '\t' + epc + '\t' + String(rssi) + '\t' + String(count) + '\n';
    if (gTypingInProgress) {
      // A type (single row or full-table export) is already running and is
      // draining the wire UART itself - queue this row instead of typing it
      // now, which would recurse back into typeCharByChar from inside its
      // own delay loop.
      gPendingLiveRows += row;
    } else {
      gTypingInProgress = true;
      typeCharByChar(row);
      gTypingInProgress = false;
      flushPendingLiveRows();
    }
  }
}

void handleScanResult(JsonDocument &doc) {
  int antenna = doc["antenna"] | 0;
  const char *epc = doc["epc"] | "";
  int rssi = doc["rssi"] | 0;
  uint32_t count = doc["count"] | 1;
  applyTagReading(static_cast<uint8_t>(antenna), epc, rssi, count);
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
  sendJsonToPi(reply);
}

void processLine(char *line, size_t length) {
  if (length == 0) return;

  JsonDocument doc;
  if (deserializeJson(doc, line, length) != DeserializationError::Ok) {
    Serial.printf("[WIRE] dropped malformed line (%u bytes)\n", static_cast<unsigned>(length));
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

void pollWireSerial() {
  while (gWireSerial.available()) {
    char c = static_cast<char>(gWireSerial.read());
    if (c == '\n') {
      processLine(gWireLineBuffer, gWireLineLength);
      gWireLineLength = 0;
      continue;
    }
    if (c == '\r') continue;
    if (gWireLineLength < kLineBufferSize - 1) {
      gWireLineBuffer[gWireLineLength++] = c;
    } else {
      gWireLineLength = 0; // line too long: drop and resync
    }
  }
}

void sendScanControl(const char *action) {
  JsonDocument doc;
  doc["type"] = "scan_control";
  doc["action"] = action;
  sendJsonToPi(doc);
}

void sendSelectGate(uint8_t gate) {
  JsonDocument doc;
  doc["type"] = "scan_control";
  doc["action"] = "select";
  doc["gate"] = gate;
  sendJsonToPi(doc);
}

// GPIO25 reflects an external channel-select signal (e.g. the antenna bank
// switch), not a manual UI control: LOW = channel 1, HIGH = channel 2.
// Every change is also announced to the Pi as a `select` command.
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
    sendSelectGate(newChannel);
    Serial.printf("[CH] switched to channel %u\n", newChannel);
  }
}

// Yellow/white only light while held down AND their button would actually
// do something - lighting them regardless would look like the press
// worked when pollClearButton()/pollExportButton() silently ignored it,
// which is confusing on the panel. Green/red are not button echoes at
// all: they show which of the two mutually-exclusive states the Pi is in
// (exactly one of them is lit at all times) - green while scanning is
// running, red otherwise.
void mirrorButtonEchoes() {
  const bool running = gStatus.scanState == 1;
  const bool yellowHeld = digitalRead(kYellowButtonPin) == LOW;
  const bool whiteHeld = digitalRead(kExportButtonPin) == LOW || digitalRead(kWhiteButtonPin) == LOW;
  digitalWrite(kYellowEchoPin, (yellowHeld && !running) ? HIGH : LOW);
  digitalWrite(kWhiteEchoPin, (whiteHeld && !running && gExportLogSize > 0) ? HIGH : LOW);
  digitalWrite(kGreenEchoPin, running ? HIGH : LOW);
  digitalWrite(kRedEchoPin, running ? LOW : HIGH);
}

// Feedback read back from those same four optocoupler outputs. Captured
// into state each loop; not yet wired to any display or outbound message.
void pollOptoFeedback() {
  gOptoFeedback[0] = digitalRead(kYellowFeedbackPin) == HIGH;
  gOptoFeedback[1] = digitalRead(kWhiteFeedbackPin) == HIGH;
  gOptoFeedback[2] = digitalRead(kGreenFeedbackPin) == HIGH;
  gOptoFeedback[3] = digitalRead(kRedFeedbackPin) == HIGH;
}

// One BLE notify per character is fragile if sent in a tight burst - pace
// every single character rather than only every field, or whole words
// come out with letters silently missing.
void typeCharByChar(const String &text) {
  for (size_t i = 0; i < text.length(); ++i) {
    gBleKeyboard.print(text[i]);
    delay(8);
    // This runs with gTypingInProgress already set by the caller, so any
    // scan_result read here queues into gPendingLiveRows instead of typing
    // (see applyTagReading) - this just keeps the wire UART's hardware RX
    // buffer from overflowing during a long batch export.
    pollWireSerial();
  }
}

// Types whatever live rows arrived while a type was already running. Only
// called right after gTypingInProgress drops back to false.
void flushPendingLiveRows() {
  if (gPendingLiveRows.length() == 0) return;
  String rows = gPendingLiveRows;
  gPendingLiveRows = "";
  gTypingInProgress = true;
  typeCharByChar(rows);
  gTypingInProgress = false;
  flushPendingLiveRows(); // more may have queued while flushing these
}

// Types the export table as tab-separated rows into whatever text field is
// focused on the paired PC, via the BLE HID keyboard link.
void typeLogOverBleKeyboard() {
  gTypingInProgress = true;
  typeCharByChar("channel\tepc\trssi_dbm\tcount\tage_s\n");
  delay(40);
  uint32_t now = millis();
  for (size_t i = 0; i < gExportLogSize; ++i) {
    const ExportEntry &entry = gExportLog[i];
    uint32_t ageS = (now - entry.lastSeenMs) / 1000;
    String row = String(entry.channel) + '\t' + entry.epc + '\t' + String(entry.rssi) + '\t' +
                 String(entry.count) + '\t' + String(ageS) + '\n';
    typeCharByChar(row);
    delay(40);
  }

  // A single dropped "key released" notification leaves the PC thinking a
  // key is still held down, which spams autorepeat forever - this one
  // call matters far more than any other, so retry it.
  for (int attempt = 0; attempt < 3; ++attempt) {
    gBleKeyboard.releaseAll();
    delay(15);
  }
  gTypingInProgress = false;
  flushPendingLiveRows();
}

// Onboard button or external "white" panel button ("export"): sends a
// `scan_control export` command to the Pi, types the current table into
// the BLE keyboard link if a PC is paired there, then puts the ESP32 into
// live-export mode - see applyTagReading() for what that does. Ends on
// the next "stop" (pollControlButtons()). Only does anything while red is
// lit (scanning stopped) and there is at least one logged row - pressing
// it while scanning is running, or with nothing logged yet, is a no-op.
void pollExportButton() {
  uint32_t now = millis();
  if (now - gLastExportButtonMs < kDebounceMs) return;
  if (digitalRead(kExportButtonPin) != LOW && digitalRead(kWhiteButtonPin) != LOW) return;

  gLastExportButtonMs = now;

  if (gStatus.scanState == 1) {
    gStatusMessage = "Stop scanning before export";
    gStatusMessageUntilMs = now + kStatusMessageMs;
    gScreenDirty = true;
    return;
  }
  if (gExportLogSize == 0) {
    gStatusMessage = "No data to export";
    gStatusMessageUntilMs = now + kStatusMessageMs;
    gScreenDirty = true;
    return;
  }

  sendScanControl("export");
  gLiveExportActive = true;

  size_t exported = gExportLogSize;
  if (gBleKeyboard.isConnected()) {
    typeLogOverBleKeyboard();
  }

  gExportLogSize = 0; // next batch starts fresh
  gStatusMessage = "Export started (" + String(exported) + " rows)";
  Serial.printf("[EXPORT] live mode on, sent %u rows\n", static_cast<unsigned>(exported));
  gStatusMessageUntilMs = now + kStatusMessageMs;
  gScreenDirty = true;
}

// External "yellow" panel button: holding it (not tapping) sends a
// `scan_control reset` command (the Pi clears its own tag tables) and
// clears the ESP32's own export log/counter, without rebooting anything.
void pollClearButton() {
  bool pressed = digitalRead(kYellowButtonPin) == LOW;
  if (!pressed) {
    gYellowPressStartMs = 0;
    gYellowHoldHandled = false;
    return;
  }
  if (gYellowPressStartMs == 0) {
    gYellowPressStartMs = millis();
    return;
  }
  if (gYellowHoldHandled) return;
  if (millis() - gYellowPressStartMs < kClearHoldMs) return;

  gYellowHoldHandled = true;

  if (gStatus.scanState == 1) {
    gStatusMessage = "Stop scanning before reset";
    gStatusMessageUntilMs = millis() + kStatusMessageMs;
    gScreenDirty = true;
    return;
  }

  sendScanControl("reset");
  gExportLogSize = 0;
  for (uint8_t channel = 1; channel <= 2; ++channel) {
    gChannels[channel] = ChannelState{};
  }
  gStatusMessage = "Log cleared";
  gStatusMessageUntilMs = millis() + kStatusMessageMs;
  gScreenDirty = true;
  Serial.println("[RESET] sent reset, cleared local log (yellow hold)");
}

// External "green"/"red" panel buttons: ask the Pi to start/stop
// scanning. Stop also ends live-export mode (see applyTagReading()).
void pollControlButtons() {
  uint32_t now = millis();
  if (now - gLastControlButtonMs < kDebounceMs) return;

  const char *action = nullptr;
  if (digitalRead(kGreenButtonPin) == LOW) {
    action = "start";
  } else if (digitalRead(kRedButtonPin) == LOW) {
    action = "stop";
  } else {
    return;
  }

  gLastControlButtonMs = now;
  sendScanControl(action);
  // Reflect the requested state on the green/red lights immediately rather
  // than waiting for the Pi's own status reply to come back over the wire
  // (or forever, if the Pi isn't connected yet) - a real status update
  // will simply confirm or correct this once it arrives.
  gStatus.scanState = (strcmp(action, "start") == 0) ? 1 : 0;
  if (strcmp(action, "stop") == 0) {
    gLiveExportActive = false;
  }
  gStatusMessage = String("Sent: ") + action;
  Serial.printf("[CTRL] sent scan_control %s\n", action);
  gStatusMessageUntilMs = now + kStatusMessageMs;
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

// One combined card for the channel's single reading (its two antennas are
// already merged before this data ever arrives).
// Compact, full-width strip (not a big card anymore) - just enough for the
// channel header and one combined line of tag info, so the LOG box below
// it can span the full width and land dead-center on the screen instead of
// squeezed into half of it beside a tall card.
void drawChannelCard(int x, int y, int w, int h, uint8_t channel) {
  const ChannelState &state = gChannels[channel];

  gScreen.drawRoundRect(x, y, w, h, 4, TFT_DARKGREY);
  gScreen.setTextDatum(TL_DATUM);
  gScreen.setTextColor(TFT_CYAN, TFT_BLACK);
  gScreen.drawString("CHANNEL " + String(channel), x + 8, y + 4, 2);

  gScreen.setTextDatum(TR_DATUM);
  gScreen.setTextColor(TFT_YELLOW, TFT_BLACK);
  gScreen.drawString(String(countTagsForChannel(channel)) + " tags", x + w - 8, y + 4, 2);
  gScreen.setTextDatum(TL_DATUM);

  if (!state.hasTag) {
    gScreen.setTextColor(TFT_DARKGREY, TFT_BLACK);
    gScreen.drawString("no tag", x + 8, y + 21, 1);
    return;
  }

  uint32_t ageMs = millis() - state.lastSeenMs;
  bool stale = ageMs > kStaleTagMs;
  gScreen.setTextColor(stale ? TFT_DARKGREY : TFT_WHITE, TFT_BLACK);

  String epcShort = state.epc;
  String line = epcShort + "  " + String(state.rssi) + "dBm x" + String(state.count) +
                "  " + String(ageMs / 1000) + "s";
  while (epcShort.length() > 4 && gScreen.textWidth(line, 1) > w - 16) {
    epcShort.remove(0, 1); // trim from the front, keep the tail visible
    line = epcShort + "  " + String(state.rssi) + "dBm x" + String(state.count) +
           "  " + String(ageMs / 1000) + "s";
  }
  gScreen.drawString(line, x + 8, y + 21, 1);
}

// Big, hard-to-miss readout of the total logged tag count (gExportLogSize,
// the same number "Logged: N" in the footer already shows) - centered in
// its own box so it reads at a glance from across the room.
void drawLogCountBox(int x, int y, int w, int h) {
  gScreen.drawRoundRect(x, y, w, h, 4, TFT_DARKGREY);
  gScreen.setTextDatum(TC_DATUM);
  gScreen.setTextColor(TFT_CYAN, TFT_BLACK);
  gScreen.drawString("LOG", x + w / 2, y + 4, 2);

  char buf[8];
  snprintf(buf, sizeof(buf), "%03u", static_cast<unsigned>(gExportLogSize));
  String countStr(buf);

  // Always zero-padded to 3 digits ("000".."512") - pick the largest font
  // that still fits the box rather than a fixed one, so it stays as big as
  // possible instead of guessing a size that might clip.
  const int candidateFonts[] = {7, 6, 4};
  int chosenFont = 4;
  for (int candidate : candidateFonts) {
    if (gScreen.textWidth(countStr, candidate) <= w - 8) {
      chosenFont = candidate;
      break;
    }
  }

  gScreen.setTextDatum(MC_DATUM);
  gScreen.setTextColor(TFT_WHITE, TFT_BLACK);
  gScreen.drawString(countStr, x + w / 2, y + 18 + (h - 18) / 2, chosenFont);
  gScreen.setTextDatum(TL_DATUM);
}

// The single dynamic footer line, in priority order: a transient export
// result/error, then a reader error, otherwise the running logged count.
void drawFooterStatusLine(int x, int y) {
  gScreen.setTextDatum(TL_DATUM);
  if (millis() < gStatusMessageUntilMs) {
    gScreen.setTextColor(TFT_YELLOW, TFT_BLACK);
    gScreen.drawString(gStatusMessage, x, y, 1);
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

  gScreen.fillCircle(220, 8, 4, gBleKeyboard.isConnected() ? TFT_GREEN : TFT_DARKGREY);

  gScreen.setTextDatum(TR_DATUM);
  gScreen.setTextColor(TFT_YELLOW, TFT_BLACK);
  gScreen.drawString("CH " + String(gCurrentChannel), 204, 2, 2);

  drawChannelCard(4, 22, 232, 32, gCurrentChannel);
  drawLogCountBox(4, 56, 232, 44);

  gScreen.setTextDatum(TL_DATUM);
  gScreen.setTextColor(TFT_SILVER, TFT_BLACK);
  gScreen.drawString(gStatus.deviceId + " - " + scanStateText(gStatus.scanState), 4, 102, 1);

  drawFooterStatusLine(4, 113);

  gScreen.setTextColor(gBleKeyboard.isConnected() ? TFT_GREEN : TFT_DARKGREY, TFT_BLACK);
  String linkLine = gBleKeyboard.isConnected() ? "PC keyboard linked" : "PC keyboard waiting...";
  if (gLiveExportActive) linkLine += "  LIVE";
  gScreen.drawString(linkLine, 4, 124, 1);

  gScreen.pushSprite(0, 0);
}

} // namespace

void setup() {
  Serial.begin(115200);

  pinMode(kExportButtonPin, INPUT_PULLUP);
  pinMode(kWhiteButtonPin, INPUT_PULLUP);
  pinMode(kYellowButtonPin, INPUT_PULLUP);
  pinMode(kGreenButtonPin, INPUT_PULLUP);
  pinMode(kRedButtonPin, INPUT_PULLUP);

  pinMode(kYellowEchoPin, OUTPUT);
  pinMode(kWhiteEchoPin, OUTPUT);
  pinMode(kGreenEchoPin, OUTPUT);
  pinMode(kRedEchoPin, OUTPUT);

  pinMode(kYellowFeedbackPin, INPUT);
  pinMode(kWhiteFeedbackPin, INPUT);
  pinMode(kGreenFeedbackPin, INPUT);
  pinMode(kRedFeedbackPin, INPUT);

  pinMode(kChannelPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kChannelPin), channelPinIsr, CHANGE);
  gCurrentChannel = (digitalRead(kChannelPin) == LOW) ? 1 : 2;

  gTft.init();
  gTft.setRotation(3); // landscape, 240x135, flipped 180 from rotation 1
  gTft.fillScreen(TFT_BLACK);
  gScreen.createSprite(240, 135);
  gScreen.setSwapBytes(true);

  gBleKeyboard.begin();
  // Default HW RX buffer is 256 bytes, which a several-second BLE-keyboard
  // export easily outlasts even with the mid-type draining above - a bigger
  // buffer gives that draining more slack before bytes actually get lost.
  gWireSerial.setRxBufferSize(4096);
  gWireSerial.begin(kWireBaud, SERIAL_8N1, kWireRxPin, kWireTxPin);

  gScreenDirty = true;
}

void loop() {
  pollWireSerial();
  pollChannelPin();
  pollExportButton();
  pollClearButton();
  pollControlButtons();
  mirrorButtonEchoes();
  pollOptoFeedback();

  static uint32_t lastRedraw = 0;
  uint32_t now = millis();
  if (gScreenDirty || now - lastRedraw > kPeriodicRedrawMs) {
    redraw();
    gScreenDirty = false;
    lastRedraw = now;
  }
}
