# MiniSat TM/TC Interface Control Document (v1.0)

All numbers live in `flight_sw/include/icd.h`; this page explains them. All multi-byte fields are
**big-endian**. All values are invented for this project.

## 1. Link layer: AX.25 / HDLC-style frame

| Field | Bytes | Value |
|---|---|---|
| Flag | 1 | `0x7E` |
| Destination address | 7 | call-sign chars shifted left 1 + SSID byte (`GROUND-0` or `MINSAT-1`) |
| Source address | 7 | same format |
| Control | 1 | `0x03` (UI frame) |
| PID | 1 | `0xF0` (no layer 3) |
| Information | n | one CCSDS packet |
| FCS | 2 | CRC-16/CCITT-FALSE over destination..information (flags excluded) |
| Flag | 1 | `0x7E` |

Bits are sent MSB first. Between the flags, a `0` is stuffed after every five `1`s. Idle line = `1`s.
Link rate: 9600 bit/s each way.

## 2. CCSDS space packet

Primary header (6 bytes): version `000` · type (0 TM / 1 TC) · secondary-header flag · APID (11 bits) ·
grouping flags `11` · sequence count (14 bits, per APID, mod 16384) · data length = bytes after the
header **minus 1** (the 2-byte packet CRC is counted inside the data field).

| Packet | After the primary header |
|---|---|
| TC, real-time (flag 0) | command ID (2) · parameters · CRC (2) |
| TC, time-tagged (flag 1) | CUC time (7) · reserved `0x00` (1) · command ID (2) · parameters · CRC (2) |
| TM (flag always 1) | CUC OBC time (7) · source data (≤ 200) · CRC (2) |

CUC time: P-field `0x2E`, 4 bytes whole seconds since mission start, 2 bytes fraction (1/65536 s).
Packet CRC: CRC-16/CCITT-FALSE over primary header + data field (check value of "123456789" = `0x29B1`).

## 3. APIDs

| TC APID | Destination | TM APID | Content |
|---|---|---|---|
| `0x100` | OBC | `0x200` | Real-time housekeeping (default, every 10 s) |
| `0x110` | ADCS | `0x201` | TC verification (ACKs) |
| `0x120` | EPS | `0x202` | On-request answer to a GET |
| `0x130` | COMMS | `0x203` | Stored housekeeping (played back by DUMP) |
| | | `0x204` | Super-sampled ADCS data (SSTM) |
| | | `0x205` | Events (FDIR, SSTM done, TLE updated) |
| | | `0x7FF` | Idle |

## 4. Commands

| APID | ID | Name | Parameters | Allowed in SAFE |
|---|---|---|---|---|
| OBC | `0x0001` | NO_OP | – | yes |
| OBC | `0x0003` | DUMP_STM | u16 max packets | yes |
| OBC | `0x0004` | SET_HK_PERIOD | u8 seconds 1..60 | yes |
| OBC | `0x0005` | TLE_UPLOAD | 2 × 69 ASCII chars (checksums verified) | no |
| OBC | `0x0006` | CLEAR_SCHEDULE | – | yes |
| OBC | `0x0007` | SET_MODE | u8 0 SAFE / 1 NOMINAL (NOMINAL needs battery ≥ threshold + 300 mV) | yes |
| ADCS | `0x0010` | GET_PARAM | u8 id 1..6 → answer on `0x202` as [u8 id][f32 value] | yes |
| ADCS | `0x0011` | SET_WHEEL | i16 rpm −6000..6000 | no |
| ADCS | `0x0012` | SSTM_START | u8 rate 1..50 Hz, u32 duration 100..60000 ms | no |
| ADCS | `0x0013` | MTQ_ONOFF | u8 0/1 | no |
| EPS | `0x0020` | SET_RAIL | u8 rail (1 payload, 2 wheel), u8 on/off | no |
| EPS | `0x0021` | SET_SAFE_MV | u16 mV 6200..7400 | yes |
| COMMS | `0x0030` | TX_POWER | u8 dBm 20..33 | no |

## 5. Verification packet (APID `0x201`, 12 bytes)

`u8 type` (1 accept OK, 2 accept FAIL, 3 exec OK, 4 exec FAIL) · `u16 TC APID` · `u16 TC sequence count` ·
`u16 command ID` · `u8 error code` · `u32 time tag (0 = real-time)`

Error codes: 2 BAD_APID · 3 DUPLICATE · 4 MODE · 5 BAD_CMD · 6 BAD_PARAM · 7 SCHED_FULL · 8 EXEC ·
9 ABORTED_SAFE · 10 TIME_PAST · 11 BAD_LENGTH. A packet with a bad CRC gets **no** ACK (it can't be trusted).

## 6. Housekeeping source data (29 bytes)

mode u8 · battery mV u16 · state of charge % u8 · solar mW u16 · load mW u16 · wheel rpm i16 ·
wheel command i16 · rails bitmask u8 · magnetometer X (×10 nT) i16 · accepted u16 · executed u16 ·
rejected u16 · schedule slots used u8 · stored packets u16 · SSTM active u8 · eclipse u8 ·
temperature (×0.1 °C) i16 · TX power dBm u8

## 7. SSTM sample (12 bytes, 16 per packet)

time ms u32 · magnetometer X (×10 nT) i16 · wheel rpm i16 · temperature (×0.1 °C) i16 · battery mV u16
