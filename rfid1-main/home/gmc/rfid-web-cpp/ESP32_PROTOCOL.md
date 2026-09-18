# ESP32 Communication Protocol

Version: 1.0  
Transport: UART or any reliable byte stream  
Recommended UART: 115200 8-N-1

This protocol is intentionally binary. Values are shown as hexadecimal bytes with spaces between bytes.

## Frame Format

All multi-byte integers use little-endian byte order.

```text
AA 55 VV FF TT SS LL LL PP PP ... CC CC
\____/    /  /  /  /  /     \_____/ \____/
 magic   ver flags type seq len       payload CRC16
```

| Field | Size | Description |
| --- | ---: | --- |
| Magic | 2 | Always `0xAA 0x55` |
| Version | 1 | Protocol version, currently `0x01` |
| Flags | 1 | Bit 0: ACK required; bit 1: response; bit 2: error; remaining bits reserved and must be zero |
| Message type | 1 | Command or event identifier |
| Sequence | 1 | Sender sequence number, incremented for every new frame |
| Payload length | 2 | Number of payload bytes, little-endian, maximum `1024` |
| Payload | 0-1024 | TLV fields, defined below |
| CRC16 | 2 | CRC-16/CCITT-FALSE over `Version` through the final payload byte; polynomial `0x1021`, initial `0xFFFF`, no reflection, transmitted little-endian |

The length field makes the protocol stream-safe. A receiver must scan for `AA 55`, read the fixed header, wait for exactly `Payload length + 2` more bytes, then verify CRC. If magic, length, version, or CRC is invalid, discard one byte and scan again for the next magic sequence.

Payload length zero is valid. Unknown message types and unknown TLV types must be ignored unless the frame has the error flag.

## Message Types

| Type | Name | Direction |
| ---: | --- | --- |
| `0x01` | `HELLO` | ESP32 -> Controller |
| `0x02` | `GET_STATUS` | Controller -> ESP32 |
| `0x03` | `STATUS` | ESP32 -> Controller |
| `0x10` | `SET_CONFIG` | Controller -> ESP32 |
| `0x11` | `CONFIG_RESULT` | ESP32 -> Controller |
| `0x20` | `START_SCAN` | Controller -> ESP32 |
| `0x21` | `STOP_SCAN` | Controller -> ESP32 |
| `0x22` | `SCAN_RESULT` | ESP32 -> Controller |
| `0x30` | `PING` | Either direction |
| `0x31` | `PONG` | Either direction |
| `0x7F` | `ERROR` | Either direction |
| `0x80-0xFF` | Application/vendor extensions | Reserved for future use |

## TLV Payload

Payload fields use this format:

```text
TT LL LL VV VV ...
\_/ \____/ \______/
 type length value
```

- TLV type is one byte.
- TLV length is an unsigned 16-bit little-endian integer.
- TLVs may appear in any order.
- Unknown TLVs must be skipped using their length.
- A message may contain multiple TLVs of the same type when the field is repeatable.
- Strings are UTF-8 without a terminating zero byte.
- Integers are little-endian.
- Boolean values are `00` or `01`.

### Common TLV Types

| TLV | Name | Value |
| ---: | --- | --- |
| `0x01` | Device ID | UTF-8 string |
| `0x02` | Firmware version | UTF-8 string |
| `0x03` | Protocol version | `uint8` |
| `0x04` | Scan mode | `uint8`: `0` stopped, `1` continuous, `2` buffer, `3` manual |
| `0x05` | Scan state | `uint8`: `0` idle, `1` running, `2` complete, `3` error |
| `0x06` | Error code | `uint16` |
| `0x07` | Error text | UTF-8 string |
| `0x08` | RSSI | signed `int8` |
| `0x09` | Antenna | `uint8` |
| `0x0A` | EPC / tag ID | raw bytes |
| `0x0B` | Tag count | `uint16` |
| `0x0C` | RF power dBm | `uint8` |
| `0x0D` | Timeout ms | `uint32` |
| `0x0E` | Timestamp ms | `uint64` |
| `0x0F` | Request ID | `uint32` |

Types `0x40-0x7F` are reserved for future standard fields. Types `0x80-0xEF` are available for application-specific fields. Types `0xF0-0xFF` are reserved for experiments and must not be required for normal operation.

## Examples

### PING with sequence 7 and no payload

CRC is calculated over `01 01 30 07 00 00`.

```text
AA 55 01 01 30 07 00 00 10 0E
```

### Controller starts continuous scanning

Payload: TLV type `04`, length `01`, value `01`.

CRC is calculated over `01 01 20 08 04 00 04 01 01`.

```text
AA 55 01 01 20 08 04 00 04 01 01 72 18
```

### ESP32 reports one tag on antenna 2

Payload contains antenna 2, RSSI `-42` (`0xD6`), tag count 1, and EPC `30 00 E2 00 11 22 33 44`.

```text
AA 55 01 02 22 21 0F 00 09 01 02 08 01 D6 0B 02 01 00 0A 08 30 00 E2 00 11 22 33 44  ....
```

Replace `....` with the little-endian CRC16 calculated over the bytes from `01` through the final EPC byte. This example intentionally shows the payload structure; implementations must calculate CRC rather than copy a fixed value.

## Reliability Rules

1. A frame with ACK-required set (`flags & 0x01`) must receive a response with the same sequence number within 500 ms.
2. Retry at most 3 times with the same sequence number. The receiver must process a duplicate sequence only once and resend the previous response.
3. A response sets the response flag (`flags & 0x02`). An error response sets the error flag and normally uses message type `0x7F`.
4. Sequence number `0x00` is valid. It wraps from `0xFF` to `0x00`.
5. Do not send a new frame while a previous ACK-required frame is awaiting its response unless the transport layer explicitly supports multiple outstanding sequences.
6. Maximum frame size is 1036 bytes with the current 1024-byte payload limit. Larger data must be chunked using future message types.

## CRC16 Reference Pseudocode

```text
crc = 0xFFFF
for byte in bytes_from_version_through_payload:
    crc = crc XOR (byte << 8)
    repeat 8 times:
        if crc & 0x8000:
            crc = ((crc << 1) XOR 0x1021) & 0xFFFF
        else:
            crc = (crc << 1) & 0xFFFF
send low_byte(crc), high_byte(crc)
```

## Compatibility and Security

- Version `0x01` receivers must reject versions greater than `0x01` with an `ERROR` response when possible.
- Never trust payload lengths without checking the maximum before allocating memory.
- Validate every TLV length against the remaining payload bytes.
- CRC provides corruption detection, not authentication or encryption. If the link can be exposed to untrusted devices, add an authenticated encryption layer outside this framing protocol.
