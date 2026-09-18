# ESP32 RFID Display Firmware

Firmware for a LilyGo TTGO T-Display (ESP32 + 1.14" ST7789 IPS display,
135x240). It talks newline-delimited JSON to the Pi over a wired RS-422
UART, and shows the latest reading for one of the system's 2 channels at
a time (each channel combines a pair of antennas in hardware before the
Pi ever sees the data, so there is one reading per channel, not per
antenna). A panel of buttons and a channel switch drive remote scan
start/stop/reset on the Pi, an export mode that types tag data into a
BLE HID keyboard link to a PC, and a set of opto-isolated echo outputs
for external equipment.

Wire format and the full GPIO pinout: [PROTOCOL.md](PROTOCOL.md).

## Hardware

Board: LilyGo TTGO T-Display (ESP32, ST7789 135x240 IPS). See
[PROTOCOL.md](PROTOCOL.md) for the complete pin table — the short version:

- **GPIO39 (RX) / GPIO13 (TX)** — wired RS-422 link to the Pi, 57600
  8-N-1, newline-delimited JSON both directions. This is the only link
  to the Pi; there is no Bluetooth between the ESP32 and the Pi.
- **GPIO25** — channel-select input from external hardware (not a manual
  button): LOW = channel 1, HIGH = channel 2. Every change is also sent
  to the Pi as `scan_control` `select`.
- **GPIO0/27, 26, 32, 33** — panel buttons: export, reset (hold), remote
  start, remote stop.
- **GPIO15/22/17/21** — opto-isolated outputs echoing those same four
  buttons.
- **GPIO35/36/2/12** (PLACEHOLDERS, confirm before wiring) — feedback
  read back from those same four optocouplers.

The ESP32 also runs a separate BLE HID keyboard ("ESP32-RFID") a PC can
pair with directly, used only during export mode - see PROTOCOL.md.

## Build and flash (PlatformIO)

```bash
pio run -e ttgo-t-display -t upload
pio device monitor -b 115200
```

`platformio.ini` configures TFT_eSPI for this exact board via build flags,
so nothing in the TFT_eSPI library itself needs to be edited.

## Testing without the real Pi backend

There is no Bluetooth-based simulator for the Pi link anymore, since it's
now a wired UART. To test without the real reader hardware, connect a
USB-to-serial adapter to GPIO39/GPIO13 (57600 8-N-1) from a PC and send
newline-delimited JSON lines matching [PROTOCOL.md](PROTOCOL.md), e.g.
with `pyserial`:

```python
import serial, time, json
ser = serial.Serial("COM5", 57600)
ser.write((json.dumps({"type": "status", "device_id": "test", "scan_state": 1}) + "\n").encode())
ser.write((json.dumps({"type": "scan_result", "antenna": 1, "epc": "3000E200112233445566", "rssi": -42, "count": 1}) + "\n").encode())
```

Drive GPIO25 high/low (or wire it to the real antenna-bank switch) to
check the display follows it, and read back whatever the ESP32 sends on
the same port to see its `scan_control`/`pong` traffic.

To test the export/BLE-keyboard side, pair a PC with the "ESP32-RFID"
Bluetooth device, focus a text field, and press the white button.

## History

An earlier version of this protocol ran over Bluetooth Classic (RFCOMM)
between the Pi and the ESP32 instead of this wired link (`tools/pi_bluetooth_json_sender.py`
and `tools/pc_bluetooth_csv_receiver.py` were built for that version and
are now obsolete, kept only for reference). It was dropped after
repeatedly resetting the ESP32 board outright (`POWERON_RESET`) whenever
its Bluetooth Classic and BLE radios were both busy at the same time
during testing. An even earlier, never-wired-up design used a binary TLV
framing instead of JSON, documented for reference in
[ESP32_PROTOCOL.md](../rfid1-main/home/gmc/rfid-web-cpp/ESP32_PROTOCOL.md).
See that file's sibling README for the Pi-side build steps.
