#!/usr/bin/env python3
"""UART bridge for Pi -> ESP32.

This helper is intended for a Raspberry Pi that sends RFID/status/heartbeat
frames to an ESP32 over UART. It uses a simple binary protocol with a fixed
magic header and an extensible TLV payload.

Example usage:
    python3 pi_uart_sender.py --port /dev/ttyAMA0 --baud 115200
"""

from __future__ import annotations

import argparse
import queue
import serial
import threading
import time
from typing import Any

# Raspberry Pi UART mapping for ESP32 link:
#   Pi GPIO14 (TXD)  -> ESP32 GPIO16 (RXD2)
#   Pi GPIO15 (RXD)  <- ESP32 GPIO17 (TXD2)
#   GND              -> GND
#
# This script assumes /dev/ttyAMA0 or /dev/serial0 is configured for UART.
# For Raspberry Pi 4/5, /dev/serial0 is commonly the preferred interface.

try:
    import RPi.GPIO as GPIO  # type: ignore
except Exception:  # pragma: no cover - non-Pi environments
    GPIO = None

MAGIC = b"\xAA\x55"
VERSION = 0x01

# Message types
MSG_HELLO = 0x01
MSG_GET_STATUS = 0x02
MSG_STATUS = 0x03
MSG_SET_CONFIG = 0x10
MSG_CONFIG_RESULT = 0x11
MSG_START_SCAN = 0x20
MSG_STOP_SCAN = 0x21
MSG_SCAN_RESULT = 0x22
MSG_PING = 0x30
MSG_PONG = 0x31
MSG_ERROR = 0x7F

# TLV tags
TLV_DEVICE_ID = 0x01
TLV_FIRMWARE_VERSION = 0x02
TLV_PROTOCOL_VERSION = 0x03
TLV_SCAN_MODE = 0x04
TLV_SCAN_STATE = 0x05
TLV_ERROR_CODE = 0x06
TLV_ERROR_TEXT = 0x07
TLV_RSSI = 0x08
TLV_ANTENNA = 0x09
TLV_EPC = 0x0A
TLV_TAG_COUNT = 0x0B
TLV_RF_POWER_DBM = 0x0C
TLV_TIMEOUT_MS = 0x0D
TLV_TIMESTAMP_MS = 0x0E
TLV_REQUEST_ID = 0x0F


class PiUartBridge:
    def __init__(self, port: str, baudrate: int = 115200, device_id: str = "pi-rfid"):
        self.port = port
        self.baudrate = baudrate
        self.device_id = device_id
        self.running = True
        self.event_queue: "queue.Queue[bytes]" = queue.Queue(maxsize=256)
        self.serial = serial.Serial(port, baudrate=baudrate, timeout=0.1)
        self.sequence = 0
        self._heartbeat_lock = threading.Lock()
        self._last_tx = 0.0

    def _next_sequence(self) -> int:
        with self._heartbeat_lock:
            self.sequence = (self.sequence + 1) & 0xFF
            return self.sequence

    def crc16_ccitt_false(self, data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    def tlv(self, tag: int, value: Any) -> bytes:
        if isinstance(value, str):
            raw = value.encode("utf-8")
        elif isinstance(value, bool):
            raw = b"\x01" if value else b"\x00"
        elif isinstance(value, int):
            raw = value.to_bytes((value.bit_length() + 7) // 8 or 1, byteorder="little", signed=False)
        elif isinstance(value, bytes):
            raw = value
        else:
            raw = str(value).encode("utf-8")
        return bytes([tag & 0xFF]) + len(raw).to_bytes(2, byteorder="little") + raw

    def build_frame(self, message_type: int, payload: bytes = b"", flags: int = 0, sequence: int | None = None) -> bytes:
        seq = self._next_sequence() if sequence is None else (sequence & 0xFF)
        payload_length = len(payload)
        crc_input = bytes([VERSION, flags, message_type, seq]) + payload_length.to_bytes(2, byteorder="little") + payload
        crc = self.crc16_ccitt_false(crc_input)
        return MAGIC + crc_input + crc.to_bytes(2, byteorder="little")

    def send_bytes(self, data: bytes) -> None:
        if not self.serial.is_open:
            return
        self.serial.write(data)
        self.serial.flush()
        self._last_tx = time.monotonic()

    def send_status(self, scan_state: int = 1, antenna_bank: int = 1, error_code: int | None = None, error_text: str | None = None) -> None:
        payload = b"".join(
            [
                self.tlv(TLV_DEVICE_ID, self.device_id),
                self.tlv(TLV_PROTOCOL_VERSION, VERSION),
                self.tlv(TLV_SCAN_STATE, scan_state),
                self.tlv(TLV_SCAN_MODE, 1 if antenna_bank == 1 else 2),
            ]
        )
        if error_code is not None:
            payload += self.tlv(TLV_ERROR_CODE, error_code)
        if error_text:
            payload += self.tlv(TLV_ERROR_TEXT, error_text)
        frame = self.build_frame(MSG_STATUS, payload)
        self.send_bytes(frame)

    def send_heartbeat(self) -> None:
        payload = b"".join([
            self.tlv(TLV_DEVICE_ID, self.device_id),
            self.tlv(TLV_PROTOCOL_VERSION, VERSION),
            self.tlv(TLV_SCAN_STATE, 0),
        ])
        frame = self.build_frame(MSG_PING, payload)
        self.send_bytes(frame)

    def send_tag_reading(self, antenna: int, epc_hex: str, rssi: int, count: int = 1, timestamp_ms: int | None = None) -> None:
        if timestamp_ms is None:
            timestamp_ms = int(time.time() * 1000)
        epc_raw = bytes.fromhex(epc_hex.replace(" ", "").replace(":", ""))
        payload = b"".join(
            [
                self.tlv(TLV_ANTENNA, antenna),
                self.tlv(TLV_EPC, epc_raw),
                self.tlv(TLV_RSSI, ((rssi + 128) & 0xFF) if rssi < 0 else rssi),
                self.tlv(TLV_TAG_COUNT, count),
                self.tlv(TLV_TIMESTAMP_MS, timestamp_ms),
            ]
        )
        frame = self.build_frame(MSG_SCAN_RESULT, payload)
        self.send_bytes(frame)

    def send_start_scan(self, antenna_bank: int = 1, timeout_ms: int = 5000) -> None:
        payload = b"".join(
            [
                self.tlv(TLV_SCAN_MODE, 1 if antenna_bank == 1 else 2),
                self.tlv(TLV_TIMEOUT_MS, timeout_ms),
            ]
        )
        self.send_bytes(self.build_frame(MSG_START_SCAN, payload))

    def send_stop_scan(self) -> None:
        self.send_bytes(self.build_frame(MSG_STOP_SCAN, b""))

    def send_error(self, error_code: int, message: str) -> None:
        payload = b"".join([
            self.tlv(TLV_ERROR_CODE, error_code),
            self.tlv(TLV_ERROR_TEXT, message),
        ])
        self.send_bytes(self.build_frame(MSG_ERROR, payload))

    def queue_event(self, frame: bytes) -> None:
        try:
            self.event_queue.put_nowait(frame)
        except queue.Full:
            try:
                self.event_queue.get_nowait()
            except queue.Empty:
                pass
            self.event_queue.put_nowait(frame)

    def send_loop(self, interval_s: float = 0.5) -> None:
        last_heartbeat = 0.0
        counter = 0
        while self.running:
            try:
                while True:
                    frame = self.event_queue.get_nowait()
                    self.send_bytes(frame)
            except queue.Empty:
                pass

            now = time.monotonic()
            if now - last_heartbeat >= interval_s:
                self.send_heartbeat()
                last_heartbeat = now
                counter += 1

            time.sleep(0.05)

    def close(self) -> None:
        self.running = False
        try:
            self.serial.close()
        except Exception:
            pass


class PiPhysicalStatus:
    """Optional GPIO wiring for LED and switch states.

The intended hardware is:
- Switch left: antenna bank 1/2
- Switch right: antenna bank 3/4
- LED left: antenna bank A active
- LED right: antenna bank B active
- Heartbeat LED: slow blink while Pi is alive

If GPIO is not available, the class simply becomes a no-op.
"""

    def __init__(self) -> None:
        self.gpio = GPIO if GPIO is not None else None
        self.led_left = 18
        self.led_right = 23
        self.heartbeat_led = 24
        self.switch_left = 17
        self.switch_right = 27
        self.initialized = False

        if self.gpio is not None:
            self.gpio.setmode(self.gpio.BCM)
            self.gpio.setup(self.led_left, self.gpio.OUT)
            self.gpio.setup(self.led_right, self.gpio.OUT)
            self.gpio.setup(self.heartbeat_led, self.gpio.OUT)
            self.gpio.setup(self.switch_left, self.gpio.IN, pull_up_down=self.gpio.PUD_DOWN)
            self.gpio.setup(self.switch_right, self.gpio.IN, pull_up_down=self.gpio.PUD_DOWN)
            self.initialized = True

    def read_antenna_bank(self) -> int:
        if not self.initialized:
            return 1
        left = self.gpio.input(self.switch_left)
        right = self.gpio.input(self.switch_right)
        if left:
            return 1
        if right:
            return 2
        return 1

    def set_bank_led(self, bank: int, on: bool) -> None:
        if not self.initialized:
            return
        if bank == 1:
            self.gpio.output(self.led_left, self.gpio.HIGH if on else self.gpio.LOW)
        elif bank == 2:
            self.gpio.output(self.led_right, self.gpio.HIGH if on else self.gpio.LOW)

    def heartbeat(self, blink: bool) -> None:
        if not self.initialized:
            return
        self.gpio.output(self.heartbeat_led, self.gpio.HIGH if blink else self.gpio.LOW)

    def cleanup(self) -> None:
        if self.gpio is not None:
            try:
                self.gpio.cleanup()
            except Exception:
                pass


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send RFID + heartbeat frames from Pi to ESP32 over UART.")
    parser.add_argument("--port", default="/dev/serial0", help="UART port; common Pi values are /dev/serial0 or /dev/ttyAMA0")
    parser.add_argument("--baud", type=int, default=115200, help="UART baudrate")
    parser.add_argument("--device-id", default="pi-rfid", help="Device ID sent inside protocol payload")
    return parser.parse_args()


def demo() -> None:
    args = _parse_args()
    bridge = PiUartBridge(port=args.port, baudrate=args.baud, device_id=args.device_id)
    status = PiPhysicalStatus()
    try:
        print(f"[Pi] UART bridge started on {args.port} @ {args.baud} baud")
        bridge.send_status(scan_state=1, antenna_bank=1)
        last_heartbeat = 0.0
        while True:
            bank = status.read_antenna_bank()
            status.set_bank_led(bank, True)
            status.set_bank_led(1 if bank == 2 else 2, False)

            now = time.monotonic()
            if now - last_heartbeat >= 1.5:
                status.heartbeat(True)
                bridge.send_status(scan_state=1, antenna_bank=bank)
                last_heartbeat = now
            else:
                status.heartbeat(False)

            # Demo tags: this is just a reference; in real code, replace with the actual
            # RFID scan data source from your reader library.
            if int(now * 10) % 25 == 0:
                epc = "30 00 00 00 01 23 45 67"
                bridge.send_tag_reading(antenna=bank, epc_hex=epc, rssi=-42, count=1)

            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[Pi] stopped")
    finally:
        bridge.close()
        status.cleanup()


if __name__ == "__main__":
    demo()
