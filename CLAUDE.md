# CLAUDE.md

This is the `cc1101-afec` library. It implements AFEC, an adaptive FEC protocol for 433 MHz, and its baselines C1–C4 on ESP32 + CC1101, using a bare-register radio driver. It is a submodule of the thesis repo `afec`. Read `docs/README.md` first.

## Status: frozen

The code is considered finished for the measurement campaign. **Do not refactor, restyle or add tests.** Only change it for a real bug found during measurement, or for a change the user explicitly asks for, such as a pin-level flag.

Any change alters the firmware under test. When you make one:
- record the new commit in the run's `meta.json`;
- update `docs/` here;
- update `../docs/06-status-and-open-items.md` in the parent repo.

## Layout

- `src/cc1101_hal.h`, `src/hal/*.h`: HAL interface, plus header-only Arduino and ESP-IDF implementations.
- `src/cc1101.*`: radio driver (4.8 kBaud GFSK, variable-length packets, ≤ 61 B).
- `src/afec_ecc.*`: Hamming SECDED, BCH t2/t5 over GF(2⁹), RS t2/t4/t8 over GF(2⁸), CRC-16.
- `src/afec_frame.*`: outer Hamming(8,4) header, inner header + CRC, ECC parity.
- `src/afec_lhs.*`: 4-bit Link Health Score LUT and the Table 1 decision.
- `src/afec_node.*`: `Node` engine (modes C1–C5, ARQ, level FSM, NACK, duty budget, relay, logging).

## Conventions

- `namespace afec` for the protocol code. The driver and HAL are in the global namespace.
- No floating point and no dynamic allocation on the hot path. Fixed-size tables only.
- The core never includes framework headers. Anything platform-specific goes through `CC1101Hal` or `afec::Platform`.
- **Log events are an interface.** `cc1101-afec-logs-parser` depends on the event and field names.
  - If you add or rename a `LOG*` call in `afec_node.cpp`, update `docs/afec_log_schema.md` and the parser.
  - Both text and JSON formats come from one macro call: `LOGx(ev, text_fmt, json_fields, args…)`.

## Build and verify

There is no standalone build here (`platformio.ini` is gitignored). Compile through a firmware repo, which builds against this checkout through a symlink:

```sh
cd ../cc1101-afec-sender && pio run -e dev
cd ../cc1101-afec-receive && pio run -e dev
```

Flashing (`-t upload`) touches the hardware: ask first.

## Committing

This is its own git repo. Commit here first, then commit the updated submodule pointer in the parent `afec` repo.
