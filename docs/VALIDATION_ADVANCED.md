# Validation — Advanced two_hall / enc_hall package

## Completed static / host checks

- Python `py_compile`: PASS for hoverlink.py, hoverserial.py, gui.py.
- SerialFeedback host struct: 78 bytes, matching packed firmware field layout.
- Synthetic telemetry checksum/decode: PASS.
- Synthetic 100-frame CSV logging: PASS, 100 data rows + header, zero sequence gaps.
- Telemetry target: 50 Hz (20 ms firmware slots).
- EEPROM parameter addresses: unique 1..60; address 0 reserved for transaction-valid key; NB_OF_VAR=64.
- EEPROM pages: final two 2-KiB pages of STM32F103RCT6 Flash; linker FLASH region reduced to 252 KiB.
- PlatformIO environments: exactly `two_hall` and `enc_hall` under common USART3-only configuration.
- Advanced-source brace balance: PASS.
- Static local-function reference scan: no unreferenced static helper found in advanced_control.c, bldc.c, main.c, comms.c.
- Removed stale `enableFin`, `curPha_max`, `runtimeGeneratedMode`, `sync_initial_raw` symbols.
- Manual calibration uses settle + true arithmetic mean + full generated-controller cold-state reset.
- `enc_hall`: TIM4 PB6/PB7 encoder AB; Left external angle; Right Hall.

## Toolchain limitation

This sandbox does not contain PlatformIO or `arm-none-eabi-gcc`, so an actual `pio run` could not be executed here. The package is therefore statically checked and protocol-tested, but **not claimed as hardware-toolchain build PASS**.

Run locally:

```bash
pio run -e two_hall
pio run -e enc_hall
```

If either compiler reports a diagnostic, use the exact first error as the next fix target rather than flashing an unbuilt image.
