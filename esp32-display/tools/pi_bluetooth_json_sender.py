#!/usr/bin/env python3
"""Reference/test Pi -> ESP32 sender over Bluetooth Classic (RFCOMM), JSON framing.

This is a standalone script for testing the ESP32 firmware in
../src/main.cpp before the real Pi-side rfid-web-cpp server is migrated
from the wired UART/binary protocol to this Bluetooth/JSON one. See
../PROTOCOL.md for the wire format.

It can also be run directly from a Windows/Linux PC (not a real Pi) to
test the ESP32 end-to-end without any Pi or reader hardware yet — see
"Testing from your own PC" in ../README.md.

Two ways to connect:

  --com COM5
      Windows: pair the ESP32 first (Settings > Bluetooth & devices >
      Add device), then find the "Outgoing" COM port it created under
      More Bluetooth options > COM Ports. Requires `pip install pyserial`.

  --mac AA:BB:CC:DD:EE:FF [--channel 1]
      Linux (including the real Raspberry Pi): connect directly over an
      RFCOMM socket after pairing with bluetoothctl. No extra dependency.

Usage:
    python3 pi_bluetooth_json_sender.py --com COM5
    python3 pi_bluetooth_json_sender.py --mac AA:BB:CC:DD:EE:FF
"""

from __future__ import annotations

import argparse
import json
import socket
import time

RFCOMM_CHANNEL = 1


class Transport:
    """Wraps either a pyserial COM port or a raw RFCOMM socket."""

    def __init__(self, com: str | None, mac: str | None, channel: int):
        self._serial = None
        self._sock = None
        self._com = com
        self._mac = mac
        self._channel = channel

    def connect(self) -> None:
        if self._com:
            import serial  # local import: only needed for the --com path

            self._serial = serial.Serial(self._com, baudrate=115200, timeout=0)
        else:
            self._sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            self._sock.connect((self._mac, self._channel))
            self._sock.settimeout(0)

    def send(self, data: bytes) -> None:
        if self._serial is not None:
            self._serial.write(data)
        else:
            self._sock.sendall(data)

    def poll_incoming(self) -> bytes:
        """Non-blocking read of whatever the ESP32 has sent back, if any."""
        try:
            if self._serial is not None:
                return self._serial.read(4096)
            return self._sock.recv(4096)
        except (BlockingIOError, OSError):
            return b""

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
        if self._sock is not None:
            self._sock.close()


class EspBluetoothJsonBridge:
    def __init__(self, transport: Transport, device_id: str = "pi-rfid"):
        self.transport = transport
        self.device_id = device_id

    def connect(self) -> None:
        self.transport.connect()

    def send(self, message: dict) -> None:
        line = json.dumps(message, separators=(",", ":")) + "\n"
        self.transport.send(line.encode("utf-8"))

    def send_status(self, scan_state: int = 1, channel: int = 1, error_text: str | None = None) -> None:
        payload = {
            "type": "status",
            "device_id": self.device_id,
            "scan_state": scan_state,
            "channel": channel,
        }
        if error_text:
            payload["error_text"] = error_text
        self.send(payload)

    def send_scan_result(self, antenna: int, epc_hex: str, rssi: int, count: int = 1) -> None:
        self.send(
            {
                "type": "scan_result",
                "antenna": antenna,
                "epc": epc_hex.replace(" ", "").replace(":", "").upper(),
                "rssi": rssi,
                "count": count,
                "ts": int(time.time() * 1000),
            }
        )

    def send_ping(self) -> None:
        self.send({"type": "ping", "ts": int(time.time() * 1000)})

    def print_incoming(self) -> None:
        """Prints anything the ESP32 sent back: pong replies, or a CSV export
        if the export button was pressed while this script is connected."""
        chunk = self.transport.poll_incoming()
        if chunk:
            print(chunk.decode("utf-8", errors="replace"), end="")

    def close(self) -> None:
        self.transport.close()


def demo(transport: Transport) -> None:
    bridge = EspBluetoothJsonBridge(transport)
    print("[Pi] connecting ...")
    bridge.connect()
    print("[Pi] connected, sending demo frames (Ctrl+C to stop)")
    try:
        bridge.send_status(scan_state=1, channel=1)
        antenna = 1
        last_heartbeat = 0.0
        while True:
            now = time.monotonic()
            epc = f"3000E200{int(now * 10) % 100000000:08d}"
            bridge.send_scan_result(antenna=antenna, epc_hex=epc, rssi=-40 - antenna, count=1)
            antenna = (antenna % 4) + 1

            if now - last_heartbeat >= 1.5:
                bridge.send_ping()
                last_heartbeat = now

            bridge.print_incoming()
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[Pi] stopped")
    finally:
        bridge.close()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send demo RFID JSON frames to the ESP32 over Bluetooth SPP.")
    parser.add_argument("--com", help="Windows COM port bound to the paired ESP32 (e.g. COM5)")
    parser.add_argument("--mac", help="ESP32 Bluetooth MAC address (Linux RFCOMM path)")
    parser.add_argument("--channel", type=int, default=RFCOMM_CHANNEL, help="RFCOMM channel for --mac (default 1)")
    args = parser.parse_args()
    if not args.com and not args.mac:
        parser.error("pass either --com (Windows) or --mac (Linux)")
    return args


if __name__ == "__main__":
    cli_args = _parse_args()
    demo(Transport(cli_args.com, cli_args.mac, cli_args.channel))
