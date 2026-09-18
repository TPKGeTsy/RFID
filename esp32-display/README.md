# ESP32 RFID Display Firmware

Firmware for a LilyGo TTGO T-Display (ESP32 + 1.14" ST7789 IPS display,
135x240). It receives RFID reader updates from the Raspberry Pi over
Bluetooth Classic (SPP) as newline-delimited JSON, and shows the latest
reading for 2 of the system's 4 antennas at a time — one "channel"
(antennas 1-2 or antennas 3-4) per screen. It also keeps a table of every
distinct tag seen and can package it as a CSV export on demand.

Wire format: [PROTOCOL.md](PROTOCOL.md).

## Hardware

- Board: LilyGo TTGO T-Display (ESP32, ST7789 135x240 IPS)
- **GPIO25** — digital channel-select input, interrupt-driven: LOW =
  channel 1 (antennas 1-2), HIGH = channel 2 (antennas 3-4). Wire this to
  the same signal that indicates which antenna bank is active; it is not
  a manual control.
- **GPIO0** (onboard top button) — export button: packages the current
  tag table as CSV and sends it over the active Bluetooth connection,
  then starts a new batch. The bottom button (GPIO35) is unused.

## Build and flash (PlatformIO)

```bash
pio run -e ttgo-t-display -t upload
pio device monitor -b 115200
```

`platformio.ini` configures TFT_eSPI for this exact board via build flags,
so nothing in the TFT_eSPI library itself needs to be edited.

## Pairing with the Raspberry Pi

The ESP32 advertises as a Bluetooth Classic device named
`ESP32-RFID-Display` and accepts one SPP (RFCOMM) client. On the Pi:

```bash
bluetoothctl
> scan on
> pair <ESP32_MAC>
> trust <ESP32_MAC>
> exit
```

Then connect an RFCOMM socket to `<ESP32_MAC>` and write newline-delimited
JSON frames per [PROTOCOL.md](PROTOCOL.md).

## Testing without the real Pi backend

`tools/pi_bluetooth_json_sender.py` is a standalone script (Linux only,
uses the stdlib `AF_BLUETOOTH`/RFCOMM socket, no extra dependency) that
sends demo `status`/`scan_result`/`ping` frames matching the protocol.
Run it from the Pi (or any Linux machine with a Bluetooth adapter) once
paired:

```bash
python3 tools/pi_bluetooth_json_sender.py --mac <ESP32_MAC>
```

It cycles through all 4 antennas so you can confirm both channels render
correctly. Drive GPIO25 high/low (or wire it to the real antenna-bank
switch) to check the display follows it, and hold GPIO0 to test a CSV
export while `tools/pc_bluetooth_csv_receiver.py` is connected instead of
the sender script.

## Exporting a batch to a PC as CSV/XLSX

1. Let the Pi's sender disconnect (or don't start it), then pair/connect
   a PC to `ESP32-RFID-Display` over Bluetooth instead.
2. On the PC, run the receiver — it waits for the export:

   ```bash
   python3 tools/pc_bluetooth_csv_receiver.py --com COM5 --out exports --xlsx
   ```

   Use `--mac <ESP32_MAC>` instead of `--com` on Linux. See the script's
   docstring for how to find the Windows COM port after pairing.
3. Press the GPIO0 button on the ESP32. The screen shows "Exported N
   rows" and the PC script saves `exports/rfid-export-<timestamp>.csv`
   (and `.xlsx` if `--xlsx` was passed).

## Relationship to the existing wired UART protocol

An earlier design had the Pi's `rfid-web-cpp` server talk to an ESP32 over
a wired UART link using a binary TLV protocol
([ESP32_PROTOCOL.md](../rfid1-main/home/gmc/rfid-web-cpp/ESP32_PROTOCOL.md)),
kept for reference only — it was never actually wired up (the code existed
but nothing called it). `rfid-web-cpp`'s `main.cpp` now implements this
Bluetooth+JSON protocol instead: it connects out to the ESP32 over
Bluetooth Classic RFCOMM and sends real `status`/`scan_result` frames from
the live reader loop. See that file's README for the Pi-side build/config
steps (`ESP32_BT_MAC`, `libbluetooth-dev`).

`tools/pi_bluetooth_json_sender.py` remains useful for testing this
firmware without the real reader hardware attached.
