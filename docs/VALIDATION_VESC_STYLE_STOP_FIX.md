# Validation — VESC-Style FOC + STOP Fix

## Source checks

PASS:

- No `rtU`, `rtY`, `rtP`, `rtDW`, `ExtU_*`, `ExtY_*`, `DW_*`, `P_*`, `RT_MODEL`, `int16_T`, `uint16_T`, or `boolean_T` remain in the five rewritten controller files.
- `bldc.c` contains no MATLAB/Simulink state symbols.
- Both `HW_PROFILE_TWO_HALL` and `HW_PROFILE_ENC_HALL` pass host GCC syntax checks for `bldc.c`, `main.c`, and `util.c` with `-Wall -Wextra -Werror` using an STM32 HAL compile stub.
- `BLDC_controller.c`, `BLDC_controller_data.c`, and `advanced_control.c` pass strict host compilation with `-Wall -Wextra -Werror`.

## STOP unit tests

PASS: `all_motor_mode_stop_tests`

- COMMUTATION: nonzero command gives drive output; zero command gives all three duty outputs zero.
- SINE: nonzero command gives drive output; zero command gives all three duty outputs zero.
- FOC: nonzero command gives drive output; zero command gives all three duty outputs zero.
- FOC zero command clears `iq_target`, `id_target`, `vq`, `vd`, q-axis PI integrator and d-axis PI integrator.
- Disabling the motor forces `MC_STATE_OFF` and zero phase duties.

PASS: `advanced_speed_stop_tests`

- Nonzero speed command produces torque request.
- A true zero host command immediately clears the outer speed-loop output.
- Repeated zero commands remain zero; a stale integral term does not reappear.

## STOP integration check

PASS by source inspection:

- `main.c` no longer arms on `-50 < cmd < 50` idle input.
- It arms only on a real host/position movement request.
- It disarms after the host command, rate-limited/filter command and inner motor targets are all zero.
- `advanced_control_reset()` is executed at final disarm.

## Protocol / EEPROM compatibility

PASS:

- `SerialFeedback`: 35 fields, same field order as `ADVANCED_BUILD_FIX02_CAL_ONLY`.
- Parameter table: same 61 names and the same EEPROM indexes `0..60` as `ADVANCED_BUILD_FIX02_CAL_ONLY`.

## Target build status

A real PlatformIO/ARM firmware link could not be executed in this environment because PlatformIO and the ARM GCC toolchain are not installed and package-network access is disabled. An installation attempt was made and failed because the environment cannot resolve PyPI.

Therefore this checkpoint is **source/host validated**, but it must still be built with the project's normal PlatformIO STM32F103 toolchain before flashing hardware.

Recommended commands on the development PC:

```bash
pio run -e two_hall
pio run -e enc_hall
```

Then bench-test STOP first with wheels unloaded and verify:

- command becomes exactly 0,
- `# MOTOR_DISARM` is printed,
- TIM1/TIM8 MOE clears after ramp-down,
- `iq/id` converge to zero,
- ISR maximum stays below the PWM-cycle budget.
