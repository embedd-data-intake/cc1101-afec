# AFEC log schema

This is the reference that `afec_node.cpp`'s logging comment points to. It was generated from the `LOG*` calls in `src/afec_node.cpp` on 2026-11-27. **If you add or change a `LOG*` call, update this file and `cc1101-afec-logs-parser` together.**

## Format

The format is chosen at compile time.

| Macro | Values | Default |
| --- | --- | --- |
| `AFEC_LOG_LEVEL` | 0 = off, 1 = ERR, 2 = WARN, 3 = INFO, 4 = DBG | 3 |
| `AFEC_LOG_FORMAT` | 0 = human-readable text, 1 = JSON Lines | 0 |

The sender and receiver firmware both build with `-D AFEC_LOG_LEVEL=4 -D AFEC_LOG_FORMAT=1`.

- **JSON (`AFEC_LOG_FORMAT=1`)**: one object per line, `{"t":<millis>,"node":<id>,"lv":"E|W|I|D","ev":"<event>", ...fields}`.
  - Values are plain integers or short ASCII strings, with no escaping.
  - Other serial output (firmware `Serial.println`, boot ROM) is mixed in. Keep only lines that start with `{`.
- **Text (`AFEC_LOG_FORMAT=0`)**: `[<millis>] N<id> <E|W|I|D> <message>`. It logs the same events at the same levels.
- **Output path**: the library has no output of its own. Logging only happens when the application sets `node.printDebug` to a printf-style function.
- **Timestamps**: `t` is `Platform::millis()` of the logging node. The two nodes' clocks are not synchronised; `parse.py --sync` estimates the offset.

## Common fields

- `t`: milliseconds since that node booted.
- `node`: `NodeConfig::id` of the node that logged the line.
- `lv`: the level, one of `E`, `W`, `I`, `D`.
- `ev`: the event name, from the tables below.

Fields used in several events:

| Field | Meaning |
| --- | --- |
| `lvl` | ECC level. 0–5 is the code ladder (see `afec_ecc.h`). 7 is `LEVEL_NONE`, meaning CRC-16 only. |
| `fb` | 4-bit feedback (0–15). In C5 it is the Link Health Score (LHS). In C4 it is the RSSI quantised to 4 bits. In C1–C3 it is 0. |
| `rssi` | dBm, from the CC1101 status byte: raw/2 − 74. |
| `lqi` | CC1101 LQI, 0–127. **Lower is better.** |
| `corr` | s, the number of bits (Hamming, BCH) or bytes (RS) the decoder repaired. |
| `ecc_t` | t, the correction capacity of the level. 0 for `LEVEL_NONE`. |
| `type` | `DATA`, `ACK` or `NACK`. |
| `seq` | 8-bit sequence number. Wraps 255 → 0. |

## Events

### Lifecycle and link

| ev | lv | Fields | When |
| --- | --- | --- | --- |
| `begin` | I | `mode` (0 = RAW/C1, 1 = ARQ/C2, 2 = HWFEC/C3, 3 = RSSI/C4, 4 = AFEC/C5), `init_lvl`, `bitrate`, `duty_pct`, `burst_ms`, `retries`, `ack_to_ms`, `pin` (255 = not pinned, else the pinned level) | `Node::begin()`. Every call resets the config, the LHS LUT and the duty budget. |
| `link_reset` | I | `peer`, `lvl` | `resetLink()`. The peer's TX and ACK levels and the hop level go back to `initialLevel`. |
| `peer_new` | D | `id`, `lvl` | A peer-table slot is allocated. |
| `peer_evict` | D | `id` | The least recently used peer is evicted. There are 8 slots. |

### Send path (sender side)

| ev | lv | Fields | When |
| --- | --- | --- | --- |
| `send` | I | `dst`, `seq`, `len`, `reliable`, `tries` | `send()` is accepted. `tries` = 1 + `maxRetries` when reliable, otherwise 1. |
| `send_rejected` | W | `dst`, `len` | The payload is too long or null. No `seq` is consumed. |
| `send_attempt` | D | `seq`, `attempt` (1-based), `tries`, `peer_lvl`, `streak`, `lhs` | Before each transmission. |
| `send_retry` | I | `seq`, `retry` (1-based), `max_retries` | Before each retransmission. |
| `send_ok` | I | `seq`, `dst`, `rtt_ms`, `fb`, `attempts` | A matching ACK arrived. `rtt_ms` is measured from the end of the TX of this attempt. |
| `send_nack` | W | `seq`, `waited_ms`, `fb` | A NACK arrived while waiting for the ACK. The frame is retransmitted at once and counts as one try. |
| `send_timeout` | W | `seq`, `waited_ms`, `timeouts` | No ACK or NACK within `ackTimeoutMs`. |
| `send_aborted` | W | `seq`, `reason` (`TOO_LONG`, `DUTY_LIMIT`, `RADIO_ERR`) | `transmit()` failed. `send()` returns that result; there are no further tries. |
| `send_fail` | E | `seq`, `dst`, `tries` | All tries were used up. Result is `NO_ACK`. |

**Per-packet outcome**: each `send` is followed by exactly one of `send_ok`, `send_fail`, `send_aborted` or `send_rejected`. `send_rejected` comes *instead of* `send`. With `reliable=0` (C1 or broadcast), a successful `transmit` returns with no closing event; only the `tx` debug line is logged.

### Level adaptation (C4/C5 only)

| ev | lv | Fields | When |
| --- | --- | --- | --- |
| `lvl_step` | I if the level changed, otherwise D | `tag`, `id`, `action`, `fb`, `from`, `to`, `streak_from`, `streak_to`, `downgrade_after` | Once per feedback value. |
| `lvl_nack_jump` | I | `peer`, `from`, `to` | NACK received on the sender. The level rises by `nackStep`, capped at 5. |
| `lvl_timeout` | I if the level changed, otherwise D | `peer`, `from`, `to` (= 5) | ACK timeout on the sender. The level is set to the maximum ("0x0 = max protect"). |

The `tag` field of `lvl_step` says which level was stepped:

- `tx`: the sender's DATA level towards `id`, driven by the `fb` in the ACK.
- `ack`: the receiver's level for its ACK replies to `id`, driven by the quality of received frames.
- `hop`: the relay level. `id` is 0.

**Filter on `tag == "tx"` for the sender's code-ladder trajectory.**

The `action` field:

- C5: `DOWNGRADE`, `HOLD`, `UPGRADE`, `NACK_UPGRADE` or `MAX_PROTECT` (Table 1).
- C4: `RSSI_UP`, `RSSI_DOWN` or `RSSI_HOLD`.
- Pinned (`pin` ≠ 255, P3 sweep): `PINNED`. `lvl_timeout` and `lvl_nack_jump` then report `to` = the pinned level.

### TX (any frame)

| ev | lv | Fields | When |
| --- | --- | --- | --- |
| `tx` | D | `type`, `src`, `dst`, `seq`, `ttl`, `flags`, `lhs`, `lvl`, `plen`, `air_bytes`, `air_us`, `budget_us` | A frame went out. `air_us` is (`air_bytes` + 9) × 8 / bitrate, doubled in C3. `budget_us` is the duty budget left after this frame. |
| `tx_clamp` | D | `req_lvl`, `lvl`, `plen` | `fitLevel()` lowered the level because the payload did not fit. |
| `tx_too_long` | W | `type`, `src`, `dst`, `seq`, `plen`, `lvl` | `encodeFrame` failed. |
| `tx_duty_limit` | W | `type`, `src`, `dst`, `seq`, `need_us`, `have_us`, `blocked` | Not enough duty-cycle budget. The frame is **not** sent. |
| `tx_radio_err` | E | `type`, `src`, `dst`, `seq` | `CC1101::send()` failed (TX timeout or FIFO underflow). |

### RX (any frame)

| ev | lv | Fields | When |
| --- | --- | --- | --- |
| `rx` | D | `type`, `src`, `dst`, `seq`, `ttl`, `plen`, `corr`, `lvl`, `ecc_t`, `rssi`, `lqi`, `fb` | A frame was decoded and is not an ACK carrying a payload. |
| `rx_ack` | D | `src`, `dst`, `ack_for`, `ttl`, `corr`, `lvl`, `ecc_t`, `rssi`, `lqi`, `fb` | An ACK was decoded. `ack_for` is the acknowledged DATA seq. `fb` here is the *local* score of the ACK frame, not the peer's feedback. |
| `rx_bad_header` | W | `len`, `rssi`, `lqi`, `total` | The outer header failed Hamming(8,4), or the frame was too short. No NACK is sent. |
| `rx_ecc_fail` | W | `len`, `lvl`, `ecc_t`, `rssi`, `lqi`, `fb`, `total` | The decoder reported more errors than t. In C4/C5 a NACK is sent, unless the node is waiting for an ACK. |
| `rx_crc_fail` | W | same as `rx_ecc_fail` | The ECC "succeeded" but CRC-16 failed: a miscorrection, or LEVEL_NONE with errors. NACK as for `rx_ecc_fail`. |
| `rx_data` | I | `src`, `seq`, `plen`, `cast` (`unicast`/`bcast`), `dup`, `fb` | A DATA frame for this node arrived. ACKed if requested; duplicates are ACKed again. |
| `rx_dup` | D | `src`, `seq` | The (src, seq) pair is already in the 32-entry seen table. |
| `rx_own` | D | `seq` | The node heard its own frame, for example relayed back. |
| `rx_nack_used` | D | `src`, `wait_seq` | A NACK was consumed by the pending `send()`. |
| `rx_nack_ignored` | D | `src` | A NACK arrived with nothing pending. |
| `ack_match` | D | `src`, `ack_for`, `fb` | The ACK matches the pending send. `fb` is the peer's feedback from the ACK's `lhs` nibble. |
| `ack_stale` | D | `src`, `ack_for`, `wait_active`, `wait_seq` | The ACK was late or unexpected, for example the ACK of a previous attempt arriving after a timeout. |

### ACK and NACK (receiver side)

| ev | lv | Fields | When |
| --- | --- | --- | --- |
| `ack_sent` | D | `dst`, `ack_for`, `fb`, `lvl` | An ACK went out. `fb` is what the sender will adapt on. |
| `ack_failed` | W | `dst`, `ack_for`, `reason` | `transmit` of the ACK failed, usually `DUTY_LIMIT`. |
| `nack_sent` | I | `fb`, `total` | A broadcast NACK went out, always at level 5 with ttl 0. |
| `nack_failed` | W | `reason` | |
| `nack_suppressed` | D | `holdoff_left_ms` | Less than `nackHoldoffMs` (150 ms) since the last NACK. |
| `nack_skipped` | D | `wait_seq` | A corrupted frame arrived while this node was waiting for an ACK. It is most likely that ACK, so no NACK is sent. |

### Relay (TTL store and forward)

| ev | lv | Fields | When |
| --- | --- | --- | --- |
| `relay_queued` | D | `type`, `src`, `dst`, `seq`, `ttl`, `delay_ms` | A frame not addressed to this node was queued with jitter (10–120 ms). |
| `relay_skip` | D | `src`, `dst`, `seq` | ttl is 0, so the frame is not relayed. |
| `relay_full` | W | `src`, `dst`, `seq`, `total` | Both relay slots are busy, so the frame is dropped. |
| `relay_fwd` | I | `type`, `src`, `dst`, `seq`, `ttl`, `lvl` | The frame was forwarded at `_hopLevel`. |
| `relay_drop` | W | `src`, `dst`, `seq`, `reason` | `transmit` of the relayed frame failed. |

## Deriving the thesis metrics from the events

These are pointers, not definitions. The definitions are in report chapter 2.

- **PDR₁**: the share of `send` events whose `send_ok` has `attempts == 1`.
- **PDR≤R**: the share with `send_ok` at all.
- **Ā**: the mean number of `tx` events of type DATA per `send`.
- **Level trajectory and occupancy**: `lvl_step` with tag `tx` (plus `lvl_nack_jump` and `lvl_timeout`). Alternatively `send_attempt.peer_lvl`, or `tx.lvl` for DATA frames.
- **Airtime and η_air**: the sum of `tx.air_us`, both node sides, DATA plus ACK plus NACK.
- **Decoder stress s/t**: `rx` on the receiver with `type == DATA`, fields `corr` and `ecc_t`.
- **Duty-cycle pressure**: `tx.budget_us` and `tx_duty_limit`.
