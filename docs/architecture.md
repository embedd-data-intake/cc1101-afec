# Architecture

This file describes the library as of commit `28d6cbf`. The library is a portable C++ implementation of AFEC, plus the non-adaptive baselines C1–C4, running on a bare-register CC1101 driver.

## Layers

```
  application (sender / receiver firmware)
        │  Node::send / poll / onData,  NodeConfig, Platform
  ┌─────▼──────────────────────────────────────────────┐
  │ afec_node     engine: modes C1–C5, ARQ, ladder FSM, │  namespace afec
  │               NACK, duty budget, relay, logging     │
  ├──────────────┬───────────────┬─────────────────────┤
  │ afec_frame   │ afec_lhs      │ afec_ecc            │  pure, no hardware
  │ outer/inner  │ 4-bit score   │ Hamming/BCH/RS,     │
  │ framing      │ LUT + Table 1 │ CRC-16              │
  └──────┬───────┴───────────────┴─────────────────────┘
         │ CC1101::send / receive (raw bytes ≤ 61, RSSI/LQI)
  ┌──────▼──────────────────┐
  │ cc1101       driver     │  global namespace, no framework headers
  ├─────────────────────────┤
  │ CC1101Hal    interface  │  SPI, CSN, MISO-ready, GDO0 IRQ, time
  └──────┬──────────────────┘
         │ hal/cc1101_hal_arduino.h  or  hal/cc1101_hal_espidf.h  (header-only)
```

- **Portability rule:** nothing above `CC1101Hal` includes a framework header.
  - The engine gets its time and randomness only through `afec::Platform`: `millis` and `random` are required, `yield` is optional.
  - The Arduino glue for `Platform` (`afec_platform_arduino.h`) lives in the firmware repos, not here.
- **No floating point** on the hot path. The only `double` is in `CC1101::setFrequency`, which runs at configuration time.
- **No dynamic allocation** anywhere. Every table is fixed size:
  - 8 peers, 32 seen (src, seq) pairs, 2 relay slots;
  - a 512 B LHS LUT;
  - GF tables: GF(2⁹) uses (1022 + 512) × 2 B ≈ 3 KB, GF(2⁸) uses (510 + 256) × 2 B ≈ 1.5 KB. Both are static and filled lazily.

## File map

| File | Lines | Role |
| --- | --- | --- |
| `src/cc1101_hal.h` | 56 | Abstract HAL. Defines `CC1101_ISR_ATTR`, which is `IRAM_ATTR` on ESP32. |
| `src/hal/cc1101_hal_arduino.h` | 89 | Arduino HAL with custom SPI pins on ESP32 and a GDO0 interrupt through `attachInterruptArg`. |
| `src/hal/cc1101_hal_espidf.h` | 137 | ESP-IDF HAL: spi_master with 1-byte polling transactions, GPIO ISR service, esp_timer. |
| `src/cc1101.h/.cpp` | 104 + 377 | Radio driver: register config, variable-length packets, TX and RX state handling, IRQ flag. |
| `src/afec_ecc.h/.cpp` | 43 + 307 | Code ladder (6 levels), GF tables, Berlekamp–Massey, Chien search, Forney, CRC-16/CCITT-FALSE. |
| `src/afec_frame.h/.cpp` | 58 + 90 | Frame encode/decode: Hamming(8,4) outer header, inner header, CRC, ECC parity. `fitLevel`. |
| `src/afec_lhs.h/.cpp` | 38 + 47 | Link Health Score: quantisers, the 8×8×8 LUT, clamping rules, Table 1 `decide()`. |
| `src/afec_node.h/.cpp` | 134 + 434 | The `Node` engine and the logging macros. |

`library.properties` declares an Arduino library (`architectures=*`). The firmware pulls it from GitHub (`env:esp32dev`) or through a symlink (`env:dev`).

## Radio configuration (`cc1101.cpp`, `RADIO_CFG`)

| Parameter | Value |
| --- | --- |
| Modulation | GFSK, about 25 kHz deviation, about 100 kHz RX bandwidth, IF about 152 kHz |
| Data rate | **4.8 kBaud**. `NodeConfig::bitrateBps` must match for the airtime accounting. |
| Preamble and sync | 4 B preamble, 16-bit sync word sent twice (30/32 bits must match). Default `0x6B1D`. |
| Packet | Variable length, ≤ 61 B payload, plus a length byte. RSSI and LQI are appended in RX. No address filter. |
| Hardware CRC | Off by default (`hwCrc=false`), so frames with bit errors reach the ECC. **Keep it off for every mode.** |
| Whitening | PN9, on |
| State machine | `MCSM1=0x00`: transmit immediately with no CCA, go to IDLE after RX and after TX. The driver relies on this. `MCSM0` auto-calibrates when leaving IDLE. |
| GDO0 | `IOCFG0=0x06`: asserted from the sync word to the end of the packet. The falling edge means a packet is done. |
| TX power | `Config::txPowerDbm` goes to PATABLE[0] and must be one of −30, −20, −15, −10, 0, 5, 7, 10 (rounds down). **It is only set in `begin()`; there is no runtime setter.** |
| RSSI | raw/2 − 74 dB (433 MHz offset) |
| Timeouts | TX 500 ms; IDLE wait 20 ms; SPI chip-ready wait 2 ms; reset 5 ms |

Driver behaviour:

- **`send`** blocks until the packet is out. It goes IDLE, flushes TX, writes the FIFO burst, strobes STX, then polls TXBYTES and MARCSTATE.
- After `send` the radio always returns to RX.
- **`receive`** is non-blocking. In IRQ mode it touches no SPI until the ISR has set `_pending`.
- A packet is complete when MARCSTATE is IDLE and RXBYTES is not zero. RXBYTES is read twice to work around the silicon erratum.
- On overflow or a malformed packet, the driver restarts RX.

## On-air frame (`afec_frame.h`)

```
 [O0][O1] [src dst seq ttl|lhs type|flags payload… crc16] [parity PB(level)]
  outer    ←──────────── inner, L bytes ──────────────→   ←─ ECC parity ─→
```

- **Outer header**: 1 info byte, `0xA0 | level`. Each nibble is protected by Hamming(8,4) SECDED, giving 2 B.
  - It sits outside the main ECC, because the receiver needs the level before it can decode.
  - The high nibble must decode to `0xA`, otherwise the frame is `BAD_HEADER`.
- **Inner header**: 5 B. They are `src`, `dst`, `seq`, `ttl<<4 | lhs`, and `type<<6 | flags`. The flags are 6 bits, and bit 0 is `ACK_REQ`.
- **CRC-16/CCITT-FALSE** covers the inner header and the payload. It catches decoder miscorrection.
- There is **no length field**: L = frame length − 2 − PB(level).
- **Limits:**
  - `AIR_MAX` is 61.
  - The payload limit at a level is `maxPayload(level)` = 61 − 2 − PB − 7.
  - `LEVEL_NONE` (7) gives 52 B, Hamming 50, BCH2 49, BCH5 46, RS2 48, RS4 44, RS8 **36**.
  - `send()` accepts at most `maxPayload(0)` = 50 B in adaptive modes and 52 B otherwise.
  - `transmit()` then calls `fitLevel()`, which **silently lowers** the level until the payload fits. That is logged as `tx_clamp` (DBG).

## Code ladder (`afec_ecc.cpp`)

| Level | Code | t | Parity | Data limit | Notes |
| --- | --- | --- | --- | --- | --- |
| 0 | Hamming SECDED | 1 bit | 2 B (9 check bits + overall parity) | 62 B | 9-bit columns that are not powers of two. Data-bit positions are bits LSB first. |
| 1 | BCH(511) t=2 | 2 bits | 3 B (18 bits) | 58 B | GF(2⁹), p(x) = x⁹ + x⁴ + 1 (0x211), shortened |
| 2 | BCH(511) t=5 | 5 bits | 6 B (45 bits) | 58 B | Generator built from the minimal polynomials of α¹ to α¹⁰ |
| 3 | RS(255) t=2 | 2 B | 4 B | len + 4 ≤ 255 | GF(2⁸), 0x11D, fcr = 0, shortened |
| 4 | RS t=4 | 4 B | 8 B | | |
| 5 | RS t=8 | 8 B | 16 B | | |

- `eccDecode` corrects in place and returns `corrected` (s).
- On failure the data is left untouched; `decodeFrame` then reports `ECC_FAIL` with s = t.
- RS re-checks its syndromes after Forney.
- The BCH generator is rebuilt on every call (`bchGen`). That is cheap at these sizes, but it is part of the measured encode and decode cost.

## Link Health Score (`afec_lhs.cpp`)

- **Formula**: Score = 0.6·Health + 0.3·SNR_norm + 0.1·LQI_norm, where Health = 1 − s/t. Each input is quantised to 3 bits (0–7).
- **LUT**: 512 entries, filled once in `begin()` as `round(15 · (60h + 30s + 10l) / 700)`.
- **Quantisers**:
  - Health: h = 7 − round(7·s/t). h = 0 when decoding failed.
  - SNR: RSSI is clamped to [−110, −50] dBm, then mapped linearly to 0–7. No noise floor is subtracted.
  - LQI: clamped to [0, 80], then 7 − round(7·lqi/80). Lower LQI is better.
- **Clamping**:
  - A failed decode gives 1–4, the NACK zone.
  - A successful decode gives 5–15.
  - 0xF is reserved for s = 0: a clean decode that scores 0xE or more is promoted to 0xF, and a decode with corrections is capped at 0xE.
  - 0x0 is never produced. It is reserved for "ACK never came".
- **`decide()` (Table 1)**:
  - 0xF → DOWNGRADE;
  - 0xA–0xE → HOLD;
  - 0x5–0x9 → UPGRADE;
  - 0x1–0x4 → NACK_UPGRADE;
  - 0x0 → MAX_PROTECT.

## Node engine (`afec_node.cpp`)

### Modes

`Mode` gives the numeric value that appears as `mode` in the `begin` log event.

| Mode | Config | TX level | ACK | Feedback in the ACK `lhs` nibble | Adapts |
| --- | --- | --- | --- | --- | --- |
| `RAW` (0) | C1 | `LEVEL_NONE` | no (`reliable` forced false) | — | no |
| `ARQ` (1) | C2 | `LEVEL_NONE` | yes, up to `maxRetries` | 0 | no |
| `HWFEC` (2) | C3 | `LEVEL_NONE` + pad to `fixedPayloadLen`; airtime ×2 | yes | 0 | no |
| `RSSI` (3) | C4 | ladder | yes | `rssiQ`: RSSI → 0–15 over [−110, −50] dBm | threshold + hysteresis |
| `AFEC` (4) | C5 | ladder | yes | LHS score | Table 1 FSM |

**C3 is incomplete.** The radio still runs variable-length packets with hardware FEC off. The fixed-length driver patch (`cc1101_fec_patch.txt`) is referenced in the code but does not exist.

### TX path: `send(dst, payload, len, reliable)`

1. Reject the call if the payload is too long or null. Assign `seq` (shared counter, also used for ACKs and NACKs) and add it to the seen table.
2. For each try `a` (from 0 to `maxRetries`):
   1. The frame's `lhs` nibble carries the last feedback received *from* `dst` (`Peer::rxFb`). This is the piggy-backed reverse-direction feedback.
   2. `transmit` at `levelFor(peer)`, which is `LEVEL_NONE` in non-adaptive modes.
   3. Busy-wait up to `ackTimeoutMs`, calling `processRx()` and `yield`.
3. Possible outcomes of a try:
   - **ACK with matching seq and src**: `stepLevel(tx)` using the fb from the ACK, then return `OK`.
   - **NACK**: `level += nackStep` (capped at 5), then retransmit at once.
   - **Timeout**: `level = MAX_LEVEL`, then retry.
   - All tries used: return `NO_ACK`.
   - `transmit` fails (`DUTY_LIMIT`, `RADIO_ERR` or `TOO_LONG`): abort at once.
4. **The level changes at most once per try, and only after the try.** A NACK or timeout affects the next retry. An ACK affects the next packet.

### RX path: `poll()`, then `processRx()` (up to 4 frames per call), then `handleAir()`

| Outcome | Action |
| --- | --- |
| BAD_HEADER / TOO_SHORT | Count it, send nothing. The sender times out. |
| ECC_FAIL / CRC_FAIL | In C4/C5, send a broadcast NACK carrying the failure score (1–4, or rssiQ in C4) at level 5, with a 150 ms holdoff. The NACK is skipped while this node waits for its own ACK. |
| OK, NACK | Consumed by a pending `send`, otherwise ignored. NACKs are never relayed. |
| OK, other frame | `from.rxFb = fb`. In C4/C5 this also steps `hop` and the `ack` level for `src`. Then dedupe. |
| OK, ACK for me | Matched against `_wait` (seq in `payload[0]`, src). Otherwise logged as stale. |
| OK, DATA for me or broadcast | ACK if requested, **including duplicates**. The callback fires only for non-duplicates. A broadcast is also relayed. |
| OK, other | Queued for relay if ttl > 0. |

### Level state per peer

- `Peer::level` is the DATA level towards the peer. It is stepped by the fb in its ACKs and by NACKs and timeouts.
- `Peer::ackLevel` is the level of the ACKs sent to the peer. It is stepped by the quality of frames received from it.
- `_hopLevel` is the relay level.
- Each of these has its own streak counter for `downgradeAfter`.

### `stepLevel` rules

- **C5**:
  - DOWNGRADE needs `downgradeAfter` consecutive 0xF values. The default is 2; the conference paper uses 1.
  - UPGRADE is +1.
  - NACK_UPGRADE is +`nackStep`.
  - MAX_PROTECT sets level 5.
  - HOLD resets the streak.
- **C4**:
  - Step up if q < `up[level]`.
  - Step down if q ≥ `up[level−1] + hys`.
  - The default `up` = {11, 9, 7, 5, 3} and `hys` = 1 are placeholders that must be calibrated and then frozen.

### Duty cycle

- **Airtime**: `airtimeUs(n)` = (n + 9) × 8 / bitrate. The 9 extra bytes are 4 B preamble, 4 B sync (the 16-bit word sent twice) and 1 B length. In C3 the result is doubled.
- **Budget**: a token bucket. It refills at `dutyPercent`% of wall time (dt_ms × 10 × pct µs) and is capped at `dutyBurstMs` (36 s). It is checked before every frame: DATA, ACK, NACK and relay.
- **Reset**: `begin()` refills the bucket.

### Relay

- A frame with ttl > 0 that is not for this node (or a broadcast DATA frame) is queued in one of 2 slots.
- The queued copy gets ttl − 1, `lhs` = local fb, and a random 10–120 ms delay.
- `pumpRelay()` sends it at `_hopLevel`.

## Defaults (`NodeConfig`)

| Field | Default | Used by |
| --- | --- | --- |
| `id` | 1 | |
| `mode` | AFEC | |
| `initialLevel` | 2 (BCH t=5) | C4/C5; `resetLink` |
| `fixedPayloadLen` | 0 | C3 padding |
| `defaultTtl` | 3 | relay |
| `bitrateBps` | 4800 | airtime |
| `dutyPercent` / `dutyBurstMs` | 10 / 36000 | duty budget |
| `ackTimeoutMs` / `maxRetries` | 400 / 3 | C2–C5 |
| `downgradeAfter` | 2 | C5 |
| `nackStep` | 2 | C4/C5 |
| `pinLevel` | `PIN_NONE` (adapt) | C4/C5; P3 sweep only |
| `relayJitterMinMs` / `relayJitterMaxMs` | 10 / 120 | relay |
| `nackHoldoffMs` | 150 | NACK |
| `lhs` | −110 / −50 dBm, `lqiWorst` 80 | LHS quantisers |
| `rssi` | −110 / −50 dBm, `up` {11, 9, 7, 5, 3}, `hys` 1 | C4 |

## Behaviours worth knowing when you read the logs

These are facts about the code, not bug reports. The code is frozen for the measurement campaign.

- **Pinned level (P3 only, since 2026-11-27).** `pinLevel` (default `PIN_NONE` = 0xFF) fixes the level of a C4/C5 node at 0..5: `begin()` starts at it, and the ACK step (`lvl_step` action `PINNED`), the NACK jump and the timeout all leave it there. The feedback (LHS or rssiQ) is still computed, sent and logged. It exists for the pinned-level characterisation sweep (main repo PLAN D1); campaign configurations C1–C5 never set it.
- **Timeout → level 5 immediately.** One lost ACK in C4/C5 sends the next retry to RS t=8. Under bursty loss this dominates the level trajectory.
- **Shared seq counter.** ACKs and NACKs consume `_seq` too, so a receiver's own seq values are not contiguous. The sender's DATA seq values are contiguous only while it sends no ACKs or NACKs. A sender in C4/C5 sends a NACK if it receives a corrupted frame while it is not waiting for an ACK, for example a late ACK. Key packets on the `send` event, not on seq arithmetic.
- **`fitLevel` clamp is silent at INFO.** A payload over 36 B can never use RS t=8. The `lvl_step` event still reports level 5, but `tx.lvl` shows what actually went on air.
- **The receiver's ACK level adapts independently** (`tag:"ack"`), so ACK airtime varies with link quality in C4/C5.
- **The NACK is a broadcast at max level**, with no seq binding. The sender attributes any NACK to its pending send.
- **The duty budget is per node and covers every frame type.** A receiver that sends many NACKs and ACKs can hit `DUTY_LIMIT` too, which appears as `ack_failed`.
