# SPI-CCP Notes — Dryer & Crystallizer Extended/Blanket Polls

Working notes captured from wire traces against real units, organized by
model. Complements `DataSheets/SPI Protocol.pdf` (formal spec) and
`DataSheets/FD-X Dryer Controller SPI Polls and Commands.pdf`. Field names
(`devId`, `addr`, `cmd1`, `cmd2`) match `lib/SpiCcp/SpiCcp.h`.

All traces below use station address `0x20` (primary) and the frame layout:

```
Poll:     10 01 <devId> <addr> <cmd1> <cmd2> 20 20 10 02 <data...> 10 03 <crcHi> <crcLo>
                DLE SOH                       RES RES  DLE STX            DLE ETX
```

(The leading `10 01`/trailing `10 03 <crc>` and `20 20` filler are the
CCP response envelope; `<data...>` is the payload documented per-poll
below.)

## 1. Device IDs seen

| Family              | DevID  |
|---------------------|--------|
| FC-Dryer            | 0x22   |
| FD-X Dryer          | 0x22   |
| FN-Dryer            | 0x22   |
| Advantage Dryer     | 0x22   |
| FC-Crystallizer     | 0x5C   |
| FN-Crystallizer     | 0x22   |

FD/FN-Crystallizer share the same poll set as the FC-Crystallizer; only
DevID/board-version bits differ (not fully captured in this trace set).

## 2. Common dryer polls (`addr = 0x20`)

These are identical across FC/FD-X/FN/Advantage dryers (DevID `0x22`).
Values are big-endian; 4-byte fields are IEEE-754 floats unless noted.

| Name                     | cmd1 | cmd2 | Response data                     | Decoded                         |
|--------------------------|------|------|------------------------------------|----------------------------------|
| Echo                     | 0x20 | 0x20 | `00 00 00 00` (varies by model)    | Loopback / heartbeat             |
| Version                  | 0x20 | 0x22 | `30 34 30 30`                       | ASCII "0400" → v04.00           |
| Process Set Point        | 0x20 | 0x30 | `<f32>`                             | Setpoint temp                   |
| Process High Delta       | 0x20 | 0x32 | `<f32>`                             | High alarm delta                |
| Process Low Delta        | 0x20 | 0x34 | `<f32>`                             | Low alarm delta (not used)      |
| Process Status           | 0x20 | 0x40 | `<u16>`                             | Status bitfield                 |
| Machine Mode             | 0x20 | 0x48 | `<u16>`                             | Mode bitfield                   |
| Machine Mode (protected) | 0x20 | 0x4A | `<u16>`                             | Mode bitfield, write-protected  |
| Process Temperature      | 0x20 | 0x70 | `<f32>`                             | Live process temp               |
| Return Temperature       | 0x20 | 0x72 | `<f32>`                             | Live return temp                |
| Dew Point                | 0x20 | 0x7C | `<f32>`                             | Dew point (often negative)      |
| Dew Point Trigger        | 0x20 | 0x80 | `<f32>`                             | Dew point alarm trigger         |

### FD-X-specific extra field

| Name | cmd1 | cmd2 | Response data | Decoded |
|------|------|------|----------------|---------|
| (unlabeled, Advantage/FD-X only) | 0x20 | 0x44 | `00 00` | u16, purpose unconfirmed |

## 3. Blanket polls — dryers

Blanket polls return several registers in one round trip. **The layout
differs per model/board**, even though several models share the same
`cmd1`/`cmd2` pair.

### 3.1 FC/FN/Advantage dryer — Blanket Poll 1 (`cmd1=0x20, cmd2=0xE0`)

Each entry in the data is `[cmd2][cmd2][value...]`, i.e. a sub-address tag
followed by that register's payload, concatenated:

```
20 30 <setpoint f32>  20 32 <hi-delta f32>  20 34 <lo-delta f32>  20 40 <status u16>  20 70 <process-temp f32>
```

| Sub-tag | Field                |
|---------|-----------------------|
| 0x30    | Process Set Point     |
| 0x32    | Process Hi Delta      |
| 0x34    | Process Lo Delta (not used) |
| 0x40    | Process Status        |
| 0x70    | Process Temperature   |

### 3.2 FC/FN/Advantage dryer — Blanket Poll 2 (`cmd1=0xED, cmd2=0x90`)

Flat array of seven `f32` values, no sub-tags:

| Index | Field         |
|-------|---------------|
| 1     | Process Temp  |
| 2     | Regen Temp    |
| 3     | Return Temp   |
| 4     | Regen Out Temp|
| 5     | Aux 1 Temp    |
| 6     | Aux 2 Temp    |
| 7     | Dew Point     |

(Example trace shows all 6 temps as the same value `42 A0 00 00`
(80.0) — likely a bench/simulated unit — plus dew point `C2 30 00 00`
(-44.0).)

### 3.3 FD-X dryer — Blanket Poll (`cmd1=0xC2, cmd2=0x2E`)

Larger, flat layout (no sub-tags), all values `u16` unless noted:

| Index | Field                  | Width |
|-------|------------------------|-------|
| 1     | Poll Status            | u8    |
| 2     | Process 1 Temp         | u16   |
| 3     | Return 1 Temp          | u16   |
| 4     | Process 2 Temp         | u16   |
| 5     | Return 2 Temp          | u16   |
| 6     | Regen Temp             | u16   |
| 7     | Regen Outlet           | u16   |
| 8     | Dryer Inlet            | u16   |
| 9     | Hopper Throat          | u16   |
| 10    | Left Bed               | u16   |
| 11    | Right Bed              | u16   |
| 12    | Hopper 1               | u16   |
| 13    | Hopper 2               | u16   |
| 14    | Hopper 3               | u16   |
| 15    | Hopper 4               | u16   |
| 16    | Hopper 5               | u16   |
| 17    | Hopper 6               | u16   |
| 18    | Dew Point              | u16 (signed, e.g. `FF 96`) |
| 19    | Return Dew Point       | u16   |
| 20    | Process Airflow (CFM)  | u32   |
| 21    | Process Airflow (FPM)  | u32   |
| 22    | Pressure Sensor 1      | u32   |
| 23    | Pressure Sensor 2      | u32   |
| 24    | Pressure Sensor 3      | u32   |
| 25    | Analog Material Level  | u8    |

> `lib/DryerFD/DryerFD.h` intentionally does **not** implement this
> blanket poll yet — see its header comment for why (ambiguous struct
> padding in the legacy MicroPython parser, unverified byte layout).

## 4. Advantage dryer — extended `0xD0`/`0xC2` register map

The Advantage board exposes many more registers under `cmd1=0xC2` and
`cmd1=0xD0` (DevID `0x22`, addr `0x20`). Payload widths below are inferred
from the captured response lengths; fields marked "???" are unconfirmed.

### 4.1 `cmd1 = 0xC2`

| cmd2 | Response data                              | Notes |
|------|---------------------------------------------|-------|
| 0x26 | `01 12 24 0d`                                | version/id-like block |
| 0x28 | `02 00 01 12 24 00 01 1a 00 0c 07 08 01 3f 00 a0 00 6d 00 6d 00 ab 00 f7 00 ab 02 9b 00 70 ff d8` | large config block |
| 0x36 | `01 12 24`                                   | version/id-like block |

### 4.2 `cmd1 = 0xD0`

| cmd2 | Response data                                            | Notes / decoded                                              |
|------|-----------------------------------------------------------|----------------------------------------------------------------|
| 0x20 | `09 be 01 00 0e`                                           | ???                                                              |
| 0x22 | `02`                                                       | ???                                                              |
| 0x24 | `00 c8`                                                    | = 200 (decimal)                                                  |
| 0x26 | `0b`                                                       | = 11 (decimal)                                                   |
| 0x28 | `03 02 03 15 12 2d 2a`                                     | possibly date/time (2003-02-03 21:45:42?)                        |
| 0x2A | `08 99 84 2d 2d 13 b8 9f 2c 65 82 e1 af 5a 31 05 9d f8 da 14 65 45 0a` | ??? (long opaque block) |
| 0x40 | `00 57 04 f8 ff ff 0f`                                      | ???                                                              |
| 0x42 | `00`                                                        | ???                                                              |
| 0x48 | `00 00 00 00 00`                                            | ???                                                              |
| 0x4C | `12 24 04 01 02 03 00 0f 28 01`                             | ??? possibly version/date/config                                 |
| 0x4E | `00 1e`                                                     | = 30 (decimal)                                                   |
| 0x50 | `01 00 00 00 00 00 00 00 00 00 00`                          | ???                                                              |
| 0x58 | `18 02 00 08 00 00 00 03 00 0a`                             | ???                                                              |
| 0x7E | `02 09 02 dc 0a`                                            | ???                                                              |
| 0x84 | `43 a0 37 82  43 37 be ae  42 ca af 12  42 ca af 12  43 2c 62 b7  43 fd c0 2a  43 2e b1 02  43 6d 86 4a` | 8× `f32`: Process Heater #1, Return #1, Process Heater #2, Return #2, Left Bed Outlet, Right Bed Outlet, Left Bed Heater, Right Bed Heater |
| 0x86 | `c0 e0 00 00  00 00 00 00`                                 | 2× `f32`: Dew Point, ??? |
| 0x88 | `00 00 4b 0b`                                               | ???                                                              |
| 0x8A | `30 00 02 74 17 ed 16 9f 16 ca 16 ca 1d f4 18 08 17 e7 17 43 01 28 42 ac 7e da 40 44 8e 97 02 d4 00 00` | Contains a `f32` "Volts" field at offset `40 44...` (≈3.07) mid-block; remainder unconfirmed |
| 0x8C | `00 96`                                                     | ???                                                              |
| 0x8E | `00 20`                                                     | ???                                                              |
| 0x90 | `02 bc`                                                     | ???                                                              |
| 0x92 | `01 e0 00 82`                                               | ???                                                              |
| 0x96 | `00 01 18 00 14 00 14`                                      | ???                                                              |
| 0xA0 | `00 00 b4 00 14 00 14`                                      | ???                                                              |
| 0xAA | `00 0d 01 68 06 54 03 84 01 90 00 c1 c8 00 00 00 78 00 78 00 1e 02 58 00 02 8a 00 32 00 0a` | ??? (long opaque block) |
| 0xCC | `0d 42 01 3e 80 3e 80 3e 80`                                | ???                                                              |
| 0xE2 | `0a`                                                        | ??? (also seen returning a raw copy of the `0x8A` block in one trace) |

## 5. Crystallizer polls (FC-Crystallizer, DevID `0x5C`, `cmd1=0xC2`)

| Name                | cmd2 | Response data                                                | Decoded / notes |
|---------------------|------|----------------------------------------------------------------|-------------------|
| Echo                | 0x20 | `00 00 00 00`                                                   | loopback          |
| Version             | 0x22 | `30 34 30 30`                                                   | ASCII "0400" → v04.00 |
| (unlabeled)         | 0x26 | `01 00 03`                                                      | ???               |
| Process Set Point   | 0x30 | `<f32>`                                                         | Setpoint temp     |
| Process High Delta  | 0x32 | `<f32>`                                                         | High alarm delta  |
| (unlabeled)         | 0x34 | `02 00 d0 02 24 82`                                             | ???               |
| (unlabeled)         | 0x36 | `01 02 00 00 00 01 2c 00 b8`                                    | ???               |
| ID Poll             | 0x38 | `03 00 00 00`                                                   | Byte0=roll response version; Byte1-2=software version (u16); Byte3=installed-hardware bitmap |
| Blanket Poll        | 0x3A | `02 00 42 00 42 00 42 00 42 00 42 00 42 00 42 00 00 00 00 00 00 00 00 00 00 00 00`| See layout below |
| (unlabeled)         | 0x3C | `01 0e 10 10 1c 20 00 c8 00 0e 10 10`                            | ???               |
| (unlabeled)         | 0x3E | `01 00 03`                                                      | ???               |
| (unlabeled)         | 0x40 | `01 00 00 0a 00 32 01 00 d4`                                    | ???               |
| (unlabeled)         | 0x42 | `01 00`                                                         | ???               |
| (unlabeled)         | 0x46 | `01 06 0e 14 17 33 20`                                          | ???               |
| (unlabeled)         | 0x48 | `01 16 00 14 00 0a 00 14 00 0a`                                 | ???               |

### 5.1 Crystallizer Blanket Poll layout (`cmd2=0x3A`)

Byte0 = poll response version (`= 2`), followed by fields whose byte
ranges are only partially confirmed by the trace:

| Bytes    | Field              |
|----------|---------------------|
| 0        | Poll response version (=2) |
| 1-2      | Process Heater       |
| 3-4      | Hopper Return        |
| 5-6      | Hopper High          |
| 7-8      | Hopper Mid-Low       |
| 9-10     | Hopper Mid-High      |
| 11-12    | Hopper Low           |
| 14-15    | Hopper Throat        |
| 16-25    | ??? (unconfirmed)    |

(Source notes conflict on the exact byte ranges for Mid-Low/Mid-High/Low
— re-verify against a live unit before relying on this table.)

## 6. Writing / updating a value (SELECT)

Example: write `333` to FC-Dryer Process Set Point (`addr 0x20`,
poll `cmd1=0x20 cmd2=0x30`, write `cmd2=0x31`).

1. **Request selection** — primary sends:
   `0x04 (EOT) 0x22 (DevID) 0x20 (Addr) 0x20 (cmd1) 0x31 (cmd2) 0x05 (ENQ)`
2. **Tributary confirms** it's ready to receive, echoing:
   `0x22 0x20 0x20 0x31 0x20 (RES) 0x10 0x30`
3. **Primary sends the data block**:
   `0x10 0x02 (DLE STX)  <value bytes>  0x10 0x03 (DLE ETX)  <crcHi> <crcLo>`
   — e.g. value `333` as `f32`: `43 A6 80 00`, giving
   `10 02 43 A6 80 00 10 03 AE 26`
4. **Tributary ACKs**: `0x10 0x31` (echoes `cmd2` after DLE).

This matches `SpiCcp::select()`'s two-round-trip flow (selection sequence,
then data block).

## 7. Model/board-version register map (per `model.py` "Model" byte)

Board reports a "Model" value; the CMD1/CMD2 addresses used for the core
registers—and the Blanket Poll layout—differ per model as follows. `0x30
/0x32/0x40/0x48` are present on every model (Process Setpoint, Process
Temp Limit, Process Status, Machine Status); differences are in the
blanket-poll `cmd1`/`cmd2` pair and its returned field order.

| Model | Board          | Blanket poll        | Blanket poll fields (in order)                                                                 |
|-------|----------------|---------------------|--------------------------------------------------------------------------------------------------|
| 0     | FC – DRY       | `0xED,0x90`         | Process Temp, Regen Temp, Return Temp, Regen Out Temp, Aux 1 Temp, Aux 2 Temp, Dew Point         |
| 1     | FD – DRY       | `0xC2,0x2E`         | See §3.3 (25-field extended layout)                                                              |
| 2     | FN – DRY       | `0xED,0x90`         | Process Temp, Regen Temp, Return Temp, Regen Out Temp, Aux 1 Temp, Aux 2 Temp, Dew Point         |
| 3     | CD – DRY       | *(no blanket poll)* | Uses discrete polls only: `0x70` Process Temp, `0x72` Return Temp, `0x7C` Dew Point               |
| 4     | FC – XTLR      | `0xED,0x90`         | Process Temp, Return Temp, Aux 1 Temp, Aux 2 Temp, *(ignore, ignore, ignore)* — Dew Point Trigger (`0x80`) **not supported** |
| 5     | FN – XTLR      | `0xED,0x90`         | Process Temp, Return Temp, Aux 1 Temp, Aux 2 Temp, *(ignore, ignore, ignore)* — Dew Point Trigger (`0x80`) **not supported** |
| 6     | (reserved)     | —                   | —                                                                                                  |

All models above use `cmd1=0xC2, cmd2=0x20` as the base poll address
except where a discrete `cmd2` is called out (e.g. Model 1/FD uses
`cmd1=0x20` for its discrete polls, per §2).
