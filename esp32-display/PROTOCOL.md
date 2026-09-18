# ESP32 Wired JSON Protocol

Version: 2.0
Transport: wired RS-422 UART, 57600 8-N-1, between the Pi and the ESP32
panel. Both sides read and write on this one link; there is no separate
Bluetooth connection between them. RS-422 module terminal wiring on the
ESP32 end: A+ -> GPIO39 (RX), A- -> GPIO13 (TX). The Pi end wires directly
into the Pi's onboard UART0 header pins - GPIO14/TXD0 (physical pin 8) and
GPIO15/RXD0 (physical pin 10) - not a USB adapter, so it needs
`enable_uart=1` and the serial console disabled on the Pi before
`/dev/serial0` exists and is free for this link (see the Pi-side README).

A separate Bluetooth link exists too, but only between the ESP32 and a
PC: a BLE HID keyboard ("ESP32-RFID") the PC pairs with directly, used
only for typing export data (see "Export mode" below). It carries no
JSON and is unrelated to this protocol.

An earlier version of this protocol ran over Bluetooth Classic (RFCOMM)
between the Pi and the ESP32 instead of this wired link. It was dropped
after repeatedly resetting the ESP32 board outright (`POWERON_RESET`)
whenever its Bluetooth Classic and BLE radios were both busy at the same
time during testing - the wired UART shares no hardware with either
radio, so it doesn't have that problem. An even earlier, never-wired-up
design used a binary TLV framing instead of JSON, documented for
reference in
[ESP32_PROTOCOL.md](../rfid1-main/home/gmc/rfid-web-cpp/ESP32_PROTOCOL.md).

## Framing

One JSON object per line (newline-delimited JSON / NDJSON):

- Each frame is a single-line, UTF-8 JSON object followed by `\n`.
- A line that fails to parse as JSON is discarded; reading resumes at the
  next `\n` - one malformed frame never desyncs the stream.
- There is no length prefix and no checksum; it's a direct wire, not a
  shared or lossy medium.
- Keep each line well under 512 bytes (the ESP32's line buffer size).

## Message types (`type` field)

| `type` | Direction | Purpose |
| --- | --- | --- |
| `status` | Pi -> ESP32 | Reader/scan state. |
| `scan_result` | Pi -> ESP32 | One tag reading, per channel. |
| `ping` | Pi -> ESP32 | Liveness check / heartbeat. |
| `pong` | ESP32 -> Pi | Reply to `ping`. |
| `error` | Pi -> ESP32 | Reader error to surface on the display. |
| `scan_control` | ESP32 -> Pi | A panel button or switch: select/start/stop/reset/export. |

Unknown `type` values are ignored so the schema can grow without breaking
older firmware.

### `status`

```json
{"type":"status","device_id":"pi-rfid","scan_state":1}
```

| Field | Type | Required | Meaning |
| --- | --- | --- | --- |
| `device_id` | string | no | Free-form Pi identifier, shown in the footer. |
| `scan_state` | int | no | `0` stopped, `1` running, `2` complete, `3` error. |
| `error_text` | string | no | Shown when `scan_state` is `3`. |

### `scan_result`

```json
{"type":"scan_result","antenna":2,"epc":"3000E200112233445566","rssi":-42,"count":1,"ts":1730000000000}
```

| Field | Type | Required | Meaning |
| --- | --- | --- | --- |
| `antenna` | int (1-4) | yes | Physical antenna number. The ESP32 derives the channel from this: antennas 1-2 -> channel 1, antennas 3-4 -> channel 2. |
| `epc` | string | yes | Tag EPC as a hex string (no spaces). |
| `rssi` | int | yes | Signed dBm, sent as a plain JSON integer. |
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

### `scan_control`

Sent by the ESP32 whenever a panel button is pressed or the channel
switch changes. Five `action` values:

```json
{"type":"scan_control","action":"select","gate":1}
{"type":"scan_control","action":"start"}
{"type":"scan_control","action":"stop"}
{"type":"scan_control","action":"reset"}
{"type":"scan_control","action":"export"}
```

| `action` | Trigger | What the Pi does |
| --- | --- | --- |
| `select` | The channel switch (GPIO25) changes. | Informational only today (logged); `gate` is `1` or `2`. |
| `start` | Green button. | Same as `POST /api/scan-control {"action":"start"}` - starts the reader loop. |
| `stop` | Red button. | Same as `POST /api/scan-control {"action":"stop"}` - stops the reader loop. Also ends the ESP32's own live-export mode (below). |
| `reset` | Yellow button, held 1.5s. | Clears every tag table the Pi holds (`readings` and `bufferReadings`), same as `/api/status-clear` + `/api/buffer-clear` combined. |
| `export` | White button. | Informational only today (logged). The ESP32-side effect is live-export mode, below. |

A fresh `status` frame follows `start`/`stop`/`reset`. Actions other than
these five are ignored.

## Display / channel model

- There are 4 antennas total, grouped into 2 channels of 2 antennas each:
  channel 1 = antennas 1-2, channel 2 = antennas 3-4. The two antennas in
  a channel are combined in hardware before the signal ever reaches the
  Pi, so the Pi only ever has **one reading per channel**, not one per
  antenna - the ESP32 mirrors that: one card on screen per channel, not
  two. The `antenna` field in `scan_result` only tells the ESP32 which
  channel a reading belongs to; it is not a distinct display slot.
- The ESP32 keeps the latest reading for both channels in memory at all
  times, plus a running count of distinct tags seen per channel (shown
  on that channel's card); the channel selector only changes which one
  is drawn on screen.
- The displayed channel is driven by an external hardware signal on
  GPIO25 (interrupt-driven, debounced), not a manual button: **LOW =
  channel 1, HIGH = channel 2**. Every change is also sent to the Pi as
  `scan_control` `select`.

## Panel buttons and GPIO pins

| GPIO | Physical control | Function |
| ---: | --- | --- |
| 25 | Channel toggle switch | Channel select (see above): sends `select`. |
| 0 | Onboard T-Display button | Export (same as GPIO27): sends `export`, see below. |
| 27 | External "white" button | Same as GPIO0. |
| 26 | External "yellow" button | **Hold for 1.5s** to send `reset`, only while red is lit (scanning stopped). A tap does nothing, and holding it while green is lit does nothing either. |
| 32 | External "green" button | Sends `start`. |
| 33 | External "red" button | Sends `stop`; also ends live-export mode. |

All of these are plain momentary-to-GND buttons read with `INPUT_PULLUP`
(active low), debounced. The panel's four illuminated pushbuttons are
wired for their own always-on lighting independent of the ESP32 - the
firmware only reads their switch contacts, it does not drive their LEDs.

### Opto-isolated echo outputs

| GPIO | Mirrors | Behavior |
| ---: | --- | --- |
| 15 | Yellow button | HIGH only while the button is held **and** red is lit - if it wouldn't actually trigger a reset (green lit), it stays LOW even while held. |
| 22 | White button | HIGH only while the button is held **and** red is lit **and** there is logged data - if it wouldn't actually trigger an export, it stays LOW even while held. |
| 17 | Running state | HIGH while `scan_state` is running (green), LOW otherwise. |
| 21 | Stopped state | LOW while `scan_state` is running, HIGH otherwise (red). |

Yellow and white deliberately only light when holding the button would
actually do something - lighting them regardless of state would look like
the press worked when it was silently ignored, which is confusing on the
panel. Green and red are **not** button echoes - they show which of two
mutually exclusive states the Pi is in, so exactly one of the two is lit
at all times: green while scanning is running, red the rest of the time
(stopped/complete/error, and on boot before the
first `status` arrives).

### Optocoupler feedback inputs (PLACEHOLDER pins)

| GPIO | Reads back | Status |
| ---: | --- | --- |
| 35 | Yellow optocoupler | Captured into state each loop; not yet wired to any display or outbound message. |
| 36 | White optocoupler | Same. |
| 2 | Green optocoupler | Same. |
| 12 | Red optocoupler | Same. |

These four pin numbers are placeholders - confirm against the real board
before wiring; several other "free" pins on this project have turned out
to already be spoken for. What the ESP32 should *do* with this feedback
is still unspecified.

## Export mode

- Every distinct `(channel, epc)` pair seen is kept in an export table
  (capacity 512 rows; once full, existing rows keep updating but new
  distinct tags stop being added). The screen's "Logged: N" counter and
  each channel card's "N tags" are both read from this table.
- Pressing the white/export button only does anything while red is lit
  (scanning stopped) **and** at least one row is logged; otherwise it's a
  no-op (a status message explains why: "Stop scanning before export" or
  "No data to export"). When it does fire: sends `scan_control export` to
  the Pi, types the table so far into the BLE keyboard link if a PC is
  paired there (tab-separated rows, one per logged tag), clears the
  table for a fresh batch, and turns on **live-export mode**.
- While live-export mode is on, every new `scan_result` from the Pi is
  typed into the BLE keyboard immediately, one tag at a time, instead of
  waiting for the next export button press.
- Pressing the red/stop button turns live-export mode back off (as well
  as sending `stop` to the Pi).
- Typing paces one character every 8ms and inserts a tab between fields
  so a focused spreadsheet cell fills in column-by-column; a final
  `releaseAll()` (retried 3 times) guards against a dropped BLE
  notification leaving a key stuck down.

There is no longer a CSV-over-Bluetooth-SPP export path (the earlier
Bluetooth Classic link to the Pi is gone) - `../tools/pc_bluetooth_csv_receiver.py`
and `../tools/pi_bluetooth_json_sender.py`, which talked to that link,
are obsolete and kept only for reference.
