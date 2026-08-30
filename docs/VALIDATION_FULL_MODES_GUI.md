# Validation — six modes / 50-Hz telemetry / terminal / GUI

## Completed checks

- Python `py_compile`: PASS for `hoverlink.py`, `hoverserial.py`, `gui.py`.
- Binary feedback struct: 58 bytes (`<H12h11HIIH`).
- Synthetic encode/decode + XOR checksum: PASS.
- Corrupted checksum rejection: PASS.
- Fixed-width formatter: PASS with signed Left/Right values.
- CSV writer: 100 synthetic consecutive frames -> 100 data rows, no dropped sequence; nominal 50-Hz sequence time 1.98 s for samples 0..99.
- `CTRL_MOD` range: 1..6.
- Legacy runtime `CTRL_TYP` parameter removed; controller type is selected internally from runtime mode.
- No `CTRL_TYP_SEL` define/reference remains.
- Current calibration: automatic boot state + manual `CALIBRATE`, 2000 samples, STOP interlock.
- `FOC_ISR_CYC/MAX` retain volatile storage; parameter pointers are `volatile void *`, so the previous discarded-qualifier design issue is removed.
- Source brace/static-function reference audit: PASS.
- USART2 / HD44780 / PCF8574 scan in active source/tools: no matches.

## Toolchain limitation

The sandbox does not contain PlatformIO, `arm-none-eabi-gcc`, `pyserial`, PyQt5 or pyqtgraph. Therefore this report does **not** claim a real `pio run`, serial-hardware run, or rendered Qt runtime PASS. PlatformIO build and hardware validation must still be run in the target development environment.

The Python protocol/CSV tests were executed without hardware by stubbing only the unavailable serial import; the real packet encode/decode, checksum, formatter and CSV code were used unchanged.
