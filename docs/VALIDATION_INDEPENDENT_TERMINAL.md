# Validation - Independent Left/Right + Interactive CSV Terminal

## Completed checks

- `python -m py_compile tools/hoverserial.py`: PASS.
- Binary feedback struct size: 50 bytes in Python, matching packed `SerialFeedback` layout in `Src/main.c`.
- Synthetic feedback checksum/decode test: PASS.
- Compact telemetry formatting test: PASS with exact expected line:

```text
cmd=0,0 rpm=0,0 iq=2,3 id=0,0 idc=2,2 hall=001,011 dcl=1893 rla=1966 rlb=1954 dcr=1940 rrb=1944 rrc=1931 cycle=2876
```

- Corrupted feedback checksum rejection: PASS.
- `start 50,50` and `drive -100 200` parser tests: PASS.
- CSV create/write/flush/close test with telemetry fields: PASS.
- Static scan of `Src/` and `tools/hoverserial.py`: no remaining `steer`, `mixerFcn`, `SPEED_COEFFICIENT`, or `STEER_COEFFICIENT` references.
- Firmware mode-change guard checks both requested commands and applied rate-limited `cmdL/cmdR`.
- STOP path uses the same `rateLimiter16()` and `filtLowPass32()` path as normal drive changes; no immediate command-state reset remains.
- Clarke/Park current telemetry scaling corrected to `current_counts / A2BIT_CONV` rather than the earlier incorrect extra `<<4` division.

## Environment limitation

The execution sandbox does not contain PlatformIO, `arm-none-eabi-gcc`, or the STM32 PlatformIO toolchain, and outbound package installation is unavailable. Therefore this package does **not** claim a real `pio run` PASS in this environment. Firmware compilation must still be run on the target development machine before flashing.
