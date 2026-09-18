#!/usr/bin/env python3
"""PC-side receiver for the ESP32 RFID CSV export.

Connects to the ESP32-RFID-Display over Bluetooth Classic SPP, waits for a
CSV_EXPORT_BEGIN ... CSV_EXPORT_END block (sent whenever the export button
on the ESP32 is pressed), and saves it as a timestamped .csv file. Pass
--xlsx to also write an .xlsx copy (requires `pip install openpyxl`).

Two ways to connect:

  --com COM5
      Windows: pair the ESP32 first (Settings > Bluetooth & devices > Add
      device > Bluetooth, or Control Panel > Devices and Printers), then
      check Settings > Bluetooth & devices > More Bluetooth options >
      COM Ports for the "Outgoing" port it created. This is the reliable
      path on Windows. Requires `pip install pyserial`.

  --mac AA:BB:CC:DD:EE:FF [--channel 1]
      Linux: connect directly over an RFCOMM socket after pairing with
      bluetoothctl. No extra dependency.

Usage:
    python3 pc_bluetooth_csv_receiver.py --com COM5 --out exports
    python3 pc_bluetooth_csv_receiver.py --mac AA:BB:CC:DD:EE:FF --out exports --xlsx
"""

from __future__ import annotations

import argparse
import datetime
import pathlib
import socket
import sys

BEGIN_MARKER = "CSV_EXPORT_BEGIN"
END_MARKER = "CSV_EXPORT_END"


class LineSource:
    """Wraps either a pyserial COM port or a raw RFCOMM socket as a line reader."""

    def __init__(self, com: str | None, mac: str | None, channel: int):
        self._serial = None
        self._sock = None
        self._buffer = b""

        if com:
            import serial  # local import: only needed for the --com path

            self._serial = serial.Serial(com, baudrate=115200, timeout=1)
        else:
            self._sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            self._sock.connect((mac, channel))

    def _read_chunk(self) -> bytes:
        if self._serial is not None:
            return self._serial.read(256)
        return self._sock.recv(256)

    def readline(self) -> str | None:
        while b"\n" not in self._buffer:
            chunk = self._read_chunk()
            if not chunk:
                return None
            self._buffer += chunk
        line, _, self._buffer = self._buffer.partition(b"\n")
        return line.decode("utf-8", errors="replace").rstrip("\r")

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
        if self._sock is not None:
            self._sock.close()


def write_xlsx(csv_path: pathlib.Path, xlsx_path: pathlib.Path) -> None:
    try:
        from openpyxl import Workbook
    except ImportError:
        print("openpyxl not installed; skipping .xlsx (pip install openpyxl)", file=sys.stderr)
        return

    workbook = Workbook()
    sheet = workbook.active
    with csv_path.open("r", encoding="utf-8") as handle:
        for line in handle:
            sheet.append(line.rstrip("\n").split(","))
    workbook.save(xlsx_path)


def receive_loop(source: LineSource, out_dir: pathlib.Path, want_xlsx: bool) -> None:
    print("[PC] connected, waiting for the ESP32 export button (Ctrl+C to stop)...")
    while True:
        line = source.readline()
        if line is None:
            print("[PC] connection closed")
            return
        if line.strip() != BEGIN_MARKER:
            continue

        rows: list[str] = []
        header_line = source.readline()
        if header_line is None:
            return
        rows.append(header_line)

        while True:
            line = source.readline()
            if line is None or line.strip() == END_MARKER:
                break
            rows.append(line)

        timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        csv_path = out_dir / f"rfid-export-{timestamp}.csv"
        csv_path.write_text("\n".join(rows) + "\n", encoding="utf-8")
        print(f"[PC] saved {csv_path} ({len(rows) - 1} rows)")

        if want_xlsx:
            xlsx_path = csv_path.with_suffix(".xlsx")
            write_xlsx(csv_path, xlsx_path)
            print(f"[PC] saved {xlsx_path}")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Receive an ESP32 RFID CSV export over Bluetooth SPP.")
    parser.add_argument("--com", help="Windows COM port bound to the paired ESP32 (e.g. COM5)")
    parser.add_argument("--mac", help="ESP32 Bluetooth MAC address (Linux RFCOMM path)")
    parser.add_argument("--channel", type=int, default=1, help="RFCOMM channel for --mac (default 1)")
    parser.add_argument("--out", default="exports", help="Output directory for the .csv/.xlsx files")
    parser.add_argument("--xlsx", action="store_true", help="Also write an .xlsx copy")
    args = parser.parse_args()
    if not args.com and not args.mac:
        parser.error("pass either --com (Windows) or --mac (Linux)")
    return args


def main() -> None:
    args = _parse_args()
    out_dir = pathlib.Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    source = LineSource(args.com, args.mac, args.channel)
    try:
        receive_loop(source, out_dir, args.xlsx)
    except KeyboardInterrupt:
        print("\n[PC] stopped")
    finally:
        source.close()


if __name__ == "__main__":
    main()
