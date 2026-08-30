# VALIDATION — VESC HOVERBOARD FINAL

Date: 2026-08-30

## Scope

Final refactor to one PlatformIO environment (`vesc_hoverboard`) with exactly seven motor files under `Src/motor/`: `mcpwm_foc.c/.h`, `mc_interface.c/.h`, `foc_math.c/.h`, and `mcconf_default.h`.

## Speed-control regression

Latest capture `hover_20260830_184816_mode2.csv` contains 137 rows at command 100/100. In that region the old firmware reports mean RPM L=426.2, R=418.4 (median L=426, R=410; max L=444, R=463).

Root cause found in the previous outer speed PID integral update: the x1000 state was multiplied by another x1000 during integration. The new update is `Ki_x1000 * error / MAIN_LOOP_HZ`, with conditional integration anti-windup. Speed command semantics are now direct mechanical RPM: in `mode control 3`, `start 100,100` means 100 RPM Left and 100 RPM Right.

Host acceptance:
- error +100 RPM -> torque command +50 (accelerate)
- error -300 RPM -> torque command -150 (brake)
- Ki-only one-step state at 200 Hz -> 50 in x1000 state (correct dt scaling)
- target 100 RPM with measured 400 RPM -> Left braking target negative; mirrored Right physical target has opposite internal sign

## ADC/DMA/ISR preservation
- `MX_ADC1_Init`: **EXACT MATCH**; current SHA256 `0c087dda47025526a5272fe4e4d84fba72347952794c66471acccbc388da28a5`; baseline SHA256 `0c087dda47025526a5272fe4e4d84fba72347952794c66471acccbc388da28a5`
- `MX_ADC2_Init`: **EXACT MATCH**; current SHA256 `e02a13888aa619f9e5ff255acfaa50df28cef64cef76b4b6cd0077c5c169b230`; baseline SHA256 `e02a13888aa619f9e5ff255acfaa50df28cef64cef76b4b6cd0077c5c169b230`
- `ISR prefix through six current samples`: **EXACT MATCH**; current SHA256 `be36a4c5fa8488483565474f961145e012628c269f54485bc8e45f492f298a10`; baseline SHA256 `be36a4c5fa8488483565474f961145e012628c269f54485bc8e45f492f298a10`
- `MX_TIM_Init`: intentional diff is confined to TIM4 encoder-AB ownership (initialized once, runtime start/stop). TIM1/TIM8 PWM and ADC-trigger configuration is unchanged. Diff lines: `5`.
- Compile-time ABI guards remain in `setup.c`: 10 x uint16 dual-ADC DMA layout, exact DCR/DCL/RLA/RLB/RRB/RRC/BAT/PA2/TEMP/PA3 offsets, and `ADC_TOTAL_CONV_TIME == 80`.
- STOP-valid ADC medians in the latest capture: `{'rla_raw': 1996, 'rlb_raw': 1959, 'rrb_raw': 1950, 'rrc_raw': 1936}`.

## Runtime mode contract

- Sensor: `1=openloop`, `2=Hall`, `3=encoder AB Left-only`.
- Comm: `1=six-step`, `2=sine PWM`, `3=SVPWM/FOC`.
- Control: `1=PWM`, `2=current`, `3=speed`, `4=position Left encoder-only`.
- No side means both. `mode sensor 3` is rejected; Right has no encoder. `mode control 4` requires explicit Left + sensor 3.
- Closed-loop speed requires Hall or Left encoder; it is rejected with open-loop mode because this port has no closed-loop sensorless observer.
- Sensor changes are accepted only while stopped and re-init ownership atomically; TIM4 is disabled unless Left encoder AB is selected.
- `live on/off` controls binary telemetry. Default OFF. A nonzero serial start frame enables live automatically, and the Python `start` command sends `live on` explicitly.

## Structure / compatibility

- Exactly seven files exist in `Src/motor/`.
- Old `advanced_control*`, `BLDC_controller*`, and `bldc*` root files are removed.
- One PlatformIO env only: `vesc_hoverboard`.
- EEPROM persistent parameter **names and addresses 1..60 are an exact match** to the validated pre-refactor table; address 0 remains the transaction key. Runtime sensor/comm/control/live selection is non-persistent.
- Binary telemetry frame layout is unchanged. `SPD_SET_L/R`, `SPD_ERR_L/R`, `SPD_OUT_L/R` are debug variables only.

## Host build/test results

GCC `-std=c11 -Wall -Wextra -Werror` PASS for `foc_math.c`, `mcpwm_foc.c`, `mc_interface.c`, `util.c`, `comms.c`, `main.c`.

- speed PID / 100-RPM overspeed braking: PASS
- mode constraints + CLI examples: PASS
- FOC/SINE/SIX_STEP/STOP core: PASS
- VESC-style `mc_interface` API: PASS
- Python tools `py_compile`: PASS

## Target-build limitation

This environment has no PlatformIO or `arm-none-eabi-gcc`, so an STM32F103 ARM link/flash is not claimed. Critical ADC init and ISR sampling prefix are instead byte-for-byte checked against the previously validated firmware, while portable changed logic is host-compiled and unit-tested.
