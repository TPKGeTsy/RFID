# RFID C++ Web Prototype

This is the first C++ web slice. It serves a minimal IP page and exposes:

`GET /api/get-ip`

`GET /api/reader-info` returns the initial receiver count (`4`).

`POST /api/update_network` accepts `{"ip":"192.168.1.10","gateway":"192.168.1.1"}` and applies the `eth0` netplan configuration.

The C++ reader opens `/dev/ttyUSB0` at 57600 baud and scans antenna codes `0x80` to `0x83`. The Radar endpoint reports a green antenna for three seconds after a valid EPC frame is received.

Scan control:

`POST /api/scan-control` with `{"action":"start"}` or `{"action":"stop"}`.

The web UI also offers `buffer` mode. It keeps the same inventory protocol but polls responses with a shorter timeout for faster batch-style reading. The reader frame parser accepts multiple tags per response and aggregates repeated EPCs with `count`.

`GET /api/status-log?antenna=all` returns the latest timestamp, RSSI, and count for each EPC detected by each antenna. Repeated reads of the same EPC on the same antenna are combined and increment `count`.

`POST /api/status-clear` clears the status log without stopping the reader.

The prototype serves HTTP on port 80 and replaces the old nginx/lighttpd web server.

Build on Raspberry Pi:

```bash
sudo apt update
sudo apt install -y g++ libbluetooth-dev
g++ -std=c++17 -O2 -Wall -Wextra -pthread main.cpp -o rfid-web-cpp -lbluetooth
```

`libbluetooth-dev` and `-lbluetooth` are needed for the ESP32 Bluetooth
bridge (BlueZ RFCOMM sockets).

The next slice can add real gateway detection, network update, and RFID reader controls.

## ESP32 display link

This server connects out over Bluetooth Classic (RFCOMM) to an ESP32
display board, which runs as the SPP server — see the `esp32-display`
project (sibling of this repo's checkout) for the firmware and
[PROTOCOL.md](../../../../esp32-display/PROTOCOL.md) for the JSON wire
format. Set the target device's MAC address via the `ESP32_BT_MAC`
environment variable (see `rfid-web-cpp.service`) before starting; the
bridge logs a warning and stays disabled if it isn't set. It automatically
reconnects if the link drops.

This replaces an earlier, never-wired-up design that sent a binary TLV
framing over a wired UART, documented for reference in
[ESP32_PROTOCOL.md](ESP32_PROTOCOL.md).