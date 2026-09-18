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
sudo apt install -y g++
g++ -std=c++17 -O2 -Wall -Wextra -pthread main.cpp -o rfid-web-cpp
```

The next slice can add real gateway detection, network update, and RFID reader controls.

## ESP32 display link

This server talks newline-delimited JSON to an ESP32 display/control
panel over a wired RS-422 UART — see the `esp32-display` project
(sibling of this repo's checkout) for the firmware and
[PROTOCOL.md](../../../../esp32-display/PROTOCOL.md) for the message
format. The wire connects directly to the Pi's onboard UART0 header pins
(GPIO14/TXD0 = physical pin 8, GPIO15/RXD0 = physical pin 10) through an
RS-422 transceiver module, not a USB adapter. The serial device path is
one of `Esp32SerialDevices` in `main.cpp` (`/dev/serial0`, `/dev/ttyAMA0`,
`/dev/ttyUSB1`); it retries opening the device and reopens the link
automatically if it drops. Set the `ESP32_SERIAL_DEVICE` environment
variable in the systemd unit to skip the guesswork once a stable path is
confirmed (e.g. from `ls -la /dev/serial/by-path/`).

`/dev/serial0` does not exist on a fresh Raspberry Pi OS image — that UART
is disabled by default and, when enabled, defaults to carrying the login
console. Before wiring this up, on the Pi:

```bash
sudo raspi-config
# Interface Options -> Serial Port
#   "Would you like a login shell to be accessible over serial?" -> No
#   "Would you like the serial port hardware to be enabled?" -> Yes
sudo reboot
```

That disables `console=serial0,115200` in `/boot/firmware/cmdline.txt` and
the `serial-getty` service, and enables `enable_uart=1` in
`/boot/firmware/config.txt`, so `/dev/serial0` shows up free for this link
after the reboot instead of fighting a login prompt over it.

## Panel indicator LEDs

Two LEDs wired straight to the Pi's own GPIO (not through the ESP32),
each with its own series resistor:

| GPIO | Physical pin | Wire | Lit when |
| ---: | ---: | --- | --- |
| 23 | 23 | brown | Always, for as long as `rfid-web-cpp` is running |
| 24 | 24 | gray | The RS-422 link to the ESP32 panel is connected (`esp32Connected`) |

Driven by mapping `/dev/gpiomem` directly (no extra library dependency) -
see `gpioInit`/`gpioSetOutput`/`gpioWrite` in `main.cpp`. The process user
needs to be in the `gpio` group for `/dev/gpiomem` to open; if it can't be
opened, both LEDs are silently disabled and a warning is logged, the rest
of the server runs normally either way.

An earlier design used Bluetooth Classic (RFCOMM) for this link instead
of a wired UART; it was dropped after repeatedly resetting the ESP32
board outright when its Bluetooth Classic and BLE radios were both busy
at once during testing. An even earlier, never-wired-up design sent a
binary TLV framing over UART, documented for reference in
[ESP32_PROTOCOL.md](ESP32_PROTOCOL.md).