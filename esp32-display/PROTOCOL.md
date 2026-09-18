# ESP32 Bluetooth + JSON Protocol

Version: 1.0
Transport: Bluetooth Classic SPP (RFCOMM). The ESP32 runs as the SPP server; the
Raspberry Pi connects to it as the SPP client.

This supersedes the wired UART + binary TLV design in
[../rfid1-main/home/gmc/rfid-web-cpp/ESP32_PROTOCOL.md](../rfid1-main/home/gmc/rfid-web-cpp/ESP32_PROTOCOL.md)
for this link. RFCOMM is already an ordered, reliable, retransmitting byte
stream, so the sequence/CRC/retry machinery from the binary protocol is not
needed here — it is replaced by the framing and message rules below.

## Discovery and pairing

- The ESP32 advertises itself as a Bluetooth Classic device named
  `ESP32-RFID-Display` and opens an SPP (RFCOMM) server on the default
  channel.
- Pair once (e.g. `bluetoothctl` on the Pi, or `sdptool`/`rfcomm bind`), then
  the Pi opens a normal RFCOMM socket to the ESP32's MAC address.
- If the link drops, the Pi is responsible for reconnecting; the ESP32
  always re-accepts a new SPP connection.

## Framing

One JSON object per line (newline-delimited JSON / NDJSON):

- Each frame is a single-line, UTF-8 JSON object followed by `\n`.
- The ESP32 discards a line that fails to parse as JSON and resumes reading
  at the next `\n` — one malformed frame never desyncs the stream.
- There is no length prefix and no checksum; RFCOMM already guarantees byte
  order and delivery.
- Keep each line well under 512 bytes (the ESP32's line buffer size).

## Message types (`type` field)

| `type` | Direction | Purpose |
| --- | --- | --- |
| `hello` | ESP32 -> Pi | Sent once when a Pi client connects: device id/firmware. |
| `status` | Pi -> ESP32 | Reader/scan state, replaces `STATUS`/`HELLO` from the old protocol. |
| `scan_result` | Pi -> ESP32 | One tag reading from one antenna. |
| `ping` | Pi -> ESP32 | Liveness check / heartbeat. |
| `pong` | ESP32 -> Pi | Reply to `ping`. |
| `error` | Pi -> ESP32 | Reader error to surface on the display. |

Unknown `type` values are ignored so the schema can grow without breaking
older firmware.

### `status`

```json
{"type":"status","device_id":"pi-rfid","scan_state":1,"channel":1}
```

| Field | Type | Required | Meaning |
| --- | --- | --- | --- |
| `device_id` | string | no | Free-form Pi identifier, shown in the footer. |
| `scan_state` | int | no | `0` stopped, `1` running, `2` complete, `3` error (same enum as the old TLV `Scan state`). |
| `channel` | int | no | Which antenna bank (1 or 2) the Pi is currently driving; informational only. |
| `error_text` | string | no | Shown when `scan_state` is `3`. |

### `scan_result`

```json
{"type":"scan_result","antenna":2,"epc":"3000E200112233445566","rssi":-42,"count":1,"ts":1730000000000}
```

| Field | Type | Required | Meaning |
| --- | --- | --- | --- |
| `antenna` | int (1-4) | yes | Physical antenna number. The ESP32 derives the channel from this: antennas 1-2 -> channel 1, antennas 3-4 -> channel 2. |
| `epc` | string | yes | Tag EPC as a hex string (no spaces). |
| `rssi` | int | yes | Signed dBm, sent as a plain JSON integer (no bias/offset needed, unlike the binary `int8` field). |
| `count` | int | no, default 1 | Repeat count for the same EPC, already aggregated Pi-side. |
| `ts` | int (epoch ms) | no | Informational; the ESP32 uses its own `millis()` at receipt time to compute "seconds ago" on screen. |

### `ping` / `pong`

```json
{"type":"ping","ts":1730000000000}
{"type":"pong","ts":1730000000000}
```

The ESP32 answers every `ping` with a `pong` immediately, echoing nothing
back but its own `millis()` timestamp. The Pi can use missed `pong`s to
detect a stalled link.

### `error`

```json
{"type":"error","code":1,"message":"reader offline"}
```

Shown transiently on the status footer.

## Display / channel model

- There are 4 antennas total, grouped into 2 channels of 2 antennas each:
  channel 1 = antennas 1-2, channel 2 = antennas 3-4.
- The ESP32 keeps the latest reading for all 4 antennas in memory at all
  times; the channel selector only changes which pair is drawn on screen.
- The displayed channel is driven by an external hardware signal on
  GPIO25 (interrupt-driven, debounced), not a manual button: **LOW =
  channel 1, HIGH = channel 2**. This is expected to track the same
  antenna-bank switch the Pi itself reads (see `PiPhysicalStatus` in
  `pi_uart_sender.py`).

## Export table and CSV output

- Every distinct `(antenna, epc)` pair seen is kept in an export table
  (capacity 512 rows; once full, existing rows keep updating but new
  distinct tags stop being added until the next export). The screen's
  "Logged: N" counter is this table's row count — i.e. "how many distinct
  tags have been read so far".
- Pressing the onboard **GPIO0** button packages the whole table as CSV
  and writes it to whatever is currently connected over Bluetooth SPP,
  then clears the table so the next press starts a fresh batch. Nothing
  is sent if no Bluetooth peer is connected.
- In practice this means: the Pi stays connected during live scanning: to
  export a batch, connect a PC over Bluetooth in place of the Pi, then
  press the button. The PC-side script is
  [`../tools/pc_bluetooth_csv_receiver.py`](../tools/pc_bluetooth_csv_receiver.py).
- Framing: a `CSV_EXPORT_BEGIN` line, a header line, one CSV row per
  logged entry, then a `CSV_EXPORT_END` line — all `\n`-terminated, same
  as the JSON frames.

```text
CSV_EXPORT_BEGIN
channel,antenna,epc,rssi_dbm,count,age_s
1,1,3000E200112233445566,-42,3,12
1,2,3000E200AABBCCDDEEFF,-51,1,4
CSV_EXPORT_END
```

| Column | Meaning |
| --- | --- |
| `channel` | 1 or 2, derived from `antenna`. |
| `antenna` | Physical antenna number, 1-4. |
| `epc` | Tag EPC hex string. |
| `rssi_dbm` | Last RSSI seen for this tag, signed dBm. |
| `count` | Last aggregated repeat count reported by the Pi for this tag. |
| `age_s` | Seconds between the last reading and the moment of export. |
