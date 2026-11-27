# Using the library

## Minimal node, Arduino on ESP32

This is what `cc1101-afec-sender/src/main.cpp` does, shortened.

```cpp
#include <SPI.h>
#include "cc1101.h"
#include "hal/cc1101_hal_arduino.h"
#include "afec_node.h"
#include "afec_platform_arduino.h"     // lives in the firmware repo: wraps millis/random/yield

CC1101HalArduino hal(SPI, PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN, 4000000, PIN_GDO0);
CC1101 radio;
afec::Node node;

static void debugPrint(const char* fmt, ...) { /* vsnprintf → Serial */ }

void setup() {
    CC1101::Config rc;            // 433.92 MHz, sync 0x6B1D, 0 dBm, whitening on, hwCrc off
    radio.begin(hal, rc);         // false = chip not answering (wiring)
    radio.enableInterrupt();      // optional; false = no GDO0, stays in polling mode

    afec::NodeConfig nc;          // defaults: C5 (AFEC), initialLevel 2, R_max 3, ACK timeout 400 ms
    nc.id = 2;
    node.printDebug = debugPrint; // set BEFORE begin() so the 'begin' event is logged
    node.begin(radio, nc, afec::arduinoPlatform());
    node.onData([](void*, uint8_t src, const uint8_t* p, uint8_t n, uint8_t fb) { /* … */ });
}

void loop() {
    node.poll();                                  // must run often on a receiver (RX, ACK, relay)
    // node.send(1, buf, len);                    // blocking up to (1+R_max)·(airtime+ackTimeout)
}
```

## Rules

- **Call `radio.begin()` before `node.begin()`.** `Node` does not configure the radio.
- **Both nodes need the same radio settings:** frequency, sync word, whitening and data rate. `NodeConfig::bitrateBps` must equal the real data rate (4800). It is only used for the airtime and duty accounting.
- **Do not call `send()` inside the `onData` callback.** The engine is single-threaded and `send()` blocks.
- **The `onData` payload pointer** is only valid during the callback.
- **Switching configuration at run time.** Call `node.begin()` again with a new `NodeConfig`. It resets:
  - the configuration;
  - the LHS LUT;
  - the hop level;
  - the duty budget, back to a full 36 s.

  It does **not** clear the peer table. Call `resetLink(peer)` as well to put the peer's levels back to `initialLevel`.
- **TX power** can only be changed by calling `radio.begin()` again with a different `Config::txPowerDbm`.
- **Mode C3** (`Mode::HWFEC`) is not functional without the missing fixed-length or hardware-FEC driver patch.

## Porting to another platform

1. **HAL.** Subclass `CC1101Hal`. Implement `spiBegin`, `spiEnd`, `spiTransfer`, `csWrite`, `nowMs`, `nowUs`, `delayMs` and `delayUs`.
   - Optionally implement `misoRead` and `irqAttach`/`irqDetach` for the GDO0 falling edge.
   - Define `CC1101_ISR_ATTR` if ISRs on your platform need a RAM attribute.
2. **Platform.** Fill an `afec::Platform` with `millis` and `random(maxExcl)`. `yield` is optional.
3. **Build.** Compile `src/*.cpp`. The HAL headers in `src/hal/` are header-only, so only the one you include is compiled.

## Compile-time switches

| Define | Default | Effect |
| --- | --- | --- |
| `AFEC_LOG_LEVEL` | 3 | 0 = off … 4 = debug. The campaign uses **4**, because the metrics need the `tx`, `rx` and `ack_*` DBG events. |
| `AFEC_LOG_FORMAT` | 0 | 1 = JSON Lines, which is what `cc1101-afec-logs-parser` expects. See [afec_log_schema.md](afec_log_schema.md). |
| `CC1101_ISR_ATTR` | `IRAM_ATTR` on ESP32 | ISR placement attribute |
| `TX_TIMEOUT_MS` (constant in `cc1101.cpp`) | 500 | Raise it if you lower the data rate. |

## Building

The library has no build setup of its own. `platformio.ini` is gitignored here. Build it through a firmware repo:

```sh
cd ../cc1101-afec-sender && pio run -e dev     # dev = symlink to this checkout
```
