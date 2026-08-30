# Validation - raw ADC telemetry + SVPWM mode 4

## Passed checks

- Python `py_compile`: PASS
- Python telemetry frame size: 42 bytes
- Firmware/Python field ordering checked: PASS
- Feedback XOR checksum including six ADC words: PASS
- Corrupted packet rejection test: PASS
- Requested compact telemetry formatting: PASS
- Edited C brace-balance check: PASS
- Generated BLDC controller is not passed `CTRL_MOD=4`: PASS
  - normal modes pass 1..3 to generated controller
  - mode 4 passes `OPEN_MODE` internally and uses separate SVPWM path
- Mode 4 does not read Hall signals in its PWM generation branch: PASS
- SVPWM output arithmetic checked over command range -1000..1000: duty commands remain inside timer range with configured 85% max modulation
- Mode 4 current protection:
  - `I_DC_MAX` DC-link chopping retained
  - `I_MOT_MAX` phase chopping added with reconstructed third phase
  - MOE off at zero command
- Existing `foc_isr_cycles` / `foc_isr_cycles_max` telemetry retained

Representative parser test:

```text
cmd=0,50 rpm=49,-51 V=42.54 T=44.2C cycle=2705 dcl=2000 rla=2001 rlb=1999 dcr=2002 rrb=2003 rrc=1998
```

## Full firmware build status

A complete `pio run` / ARM GCC build cannot be executed in this sandbox because PlatformIO and the ARM embedded toolchain are not installed/available here. Therefore this package is **not claimed as hardware-build PASS**. Run `pio run -e VARIANT_USART` on the development machine before flashing.

## Hardware test recommendation

For first SVPWM mode-4 testing, use a current-limited supply or safely unloaded wheels and begin with low command magnitude. Open-loop sensorless SVPWM cannot guarantee rotor synchronism at low speed or high load; if the rotor loses synchronism, reduce requested electrical frequency/load or adjust the V/f modulation constants.
