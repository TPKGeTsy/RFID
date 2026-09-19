# RFID Gate Scanning System

An RFID inventory/gate-tracking system: a Raspberry Pi reads UHF RFID tags
from up to 4 antennas (2 gates x 2 antennas each), and an ESP32 control
panel lets an operator pick a gate, start/stop/reset/export scans, and see
live tag counts on its own screen - all without needing a laptop open.

## Components

- **[`rfid1-main/home/gmc/rfid-web-cpp`](rfid1-main/home/gmc/rfid-web-cpp/README.md)**
  - The Pi-side backend: a small C++ HTTP server that talks to the RFID
    reader over serial, serves a web UI, and bridges to the ESP32 panel.
- **[`esp32-display`](esp32-display/README.md)**
  - Firmware for a LilyGo TTGO T-Display (ESP32 + screen) that is the
    physical control panel: gate switch, start/stop/reset/export buttons,
    opto-isolated outputs for external equipment, and a BLE HID keyboard
    link so scanned tags can be typed straight into a spreadsheet on a PC.

## How it fits together

```
RFID reader --serial--> Pi (rfid-web-cpp) --wired RS-422 UART--> ESP32 panel --BLE HID--> PC (Excel etc.)
                              |
                          HTTP web UI
```

The Pi and ESP32 talk newline-delimited JSON over a wired RS-422 link
(no Bluetooth between them - see
[esp32-display/PROTOCOL.md](esp32-display/PROTOCOL.md) for why and for the
full message format). The ESP32 separately runs a BLE keyboard that a PC
pairs with directly, used only for typing exported tag data.

## Status

- Pi backend: deployed and running on the target Pi, reading the RFID
  reader over USB serial and serving the web UI.
- ESP32 panel: firmware built and tested against a simulated Pi link;
  wired RS-422 link to the Pi is connected and working (`/dev/serial0` via
  the Pi's onboard UART0 header pins, GPIO14/15).
- Optocoupler feedback inputs on the ESP32 (see PROTOCOL.md) are wired but
  their pin numbers are still placeholders, and what to do with that
  feedback is still unspecified.

## Where to start

- Setting up or redeploying the Pi backend: [rfid1-main/home/gmc/rfid-web-cpp/README.md](rfid1-main/home/gmc/rfid-web-cpp/README.md)
- Building/flashing the ESP32 panel: [esp32-display/README.md](esp32-display/README.md)
- Wire protocol and full GPIO pinout: [esp32-display/PROTOCOL.md](esp32-display/PROTOCOL.md)
