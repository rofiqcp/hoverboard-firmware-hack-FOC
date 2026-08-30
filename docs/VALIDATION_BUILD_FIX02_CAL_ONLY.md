# BUILD_FIX02_CAL_ONLY

Baseline: ADVANCED_BUILD_FIX01. All advanced features are preserved.

Only functional source file changed: `Src/bldc.c`.

Manual calibration now starts from exactly the same offset initial state as boot (`2000` for all six ADC offset channels, `offsetcount=0`) and both boot/manual use the same proven 2000-iteration ISR calibration path from FULL_MODES_GUI FIX rtP_Right.

The advanced calibration state machine (`CAL_SETTLE/CAL_ACCUMULATE/CAL_WAIT_RESET`) and post-calibration cold reset were removed because they were the calibration behavior that differed from the known-good FULL_MODES_GUI baseline. Compatibility functions remain so main-loop integration is unchanged.

Preserved unchanged from ADVANCED_BUILD_FIX01:
- `two_hall` and `enc_hall` PlatformIO environments
- advanced encoder/position-control files
- RAM/EEPROM tuning
- GUI and terminal tools
- telemetry and CSV
- modes and USART3 integration

Validation:
- Source-tree comparison against ADVANCED_BUILD_FIX01: only `Src/bldc.c` differs functionally.
- Python tools syntax: PASS.
- C brace balance: PASS.
- No stale advanced calibration state-machine symbols: PASS.
- PlatformIO/ARM GCC unavailable in sandbox; hardware build PASS is not claimed.
