# Version Diff Audit — STOP Regression and VESC-Style Refactor

## Scope

Archive audited: `hoverboard-firmware-hack-FOC(1).zip` containing seven versions.

Files requested for controller refactor:

- `Src/advanced_control.c`
- `Src/advanced_control.h`
- `Src/BLDC_controller_data.c`
- `Src/BLDC_controller.c`
- `Src/BLDC_controller.h`

Integration file:

- `Src/bldc.c`

Reference for the final base: `hoverboard-firmware-hack-FOC_v3_ADVANCED_BUILD_FIX02_CAL_ONLY` with the STOP behavior restored from `hoverboard-firmware-hack-FOC_v3_ROBUST_CAL_SERIAL`.

## Diff result

The controller source was not the source of the version-to-version STOP regression:

- `BLDC_controller.c` SHA-256 prefix was identical in all seven versions: `f4f303fe7a1d`.
- `BLDC_controller.h` was identical in all seven versions: `45a7a3e48252`.
- `BLDC_controller_data.c` was identical in all seven versions: `ea487c1438a3`.
- `advanced_control.c/.h` were identical in the four advanced versions where they exist.
- `bldc.c` and `main.c` are where the later integration behavior diverged.

## STOP regression found

`ADVANCED_BUILD_FIX02_CAL_ONLY` changed the main-loop arm condition to arm the power stage while both host commands were inside the neutral band:

```c
input1[0].cmd > -50 && input1[0].cmd < 50 &&
input2[0].cmd > -50 && input2[0].cmd < 50
```

That is the opposite of the desired arm-on-motion behavior. In the same version, the `MOTOR_DISARM` block that existed in `ROBUST_CAL_SERIAL` after the complete ramp-down was removed.

The final source restores these rules:

1. Arm only when there is a real movement request.
2. Keep the bridge active only while the existing rate-limited command is decaying.
3. When host command, filtered/ramped command and inner controller output are all exactly zero, disarm the bridge.
4. Reset outer speed/position PID state at the final STOP point.
5. Inside the FOC core, a true zero setpoint forces `iq_target`, `id_target`, `vq`, `vd` and PI integrators to zero and produces zero phase duty.

## VESC-style controller rewrite

The Simulink/MATLAB generated controller implementation has been replaced by a compact handwritten controller using VESC-style naming and state organization:

- `mc_state`
- `mc_control_mode`
- `mc_motor_type`
- `mc_configuration`
- `motor_all_state_t`
- `m_motor_1`, `m_motor_2`
- `m_motor_state`
- `m_iq_pi`, `m_id_pi`
- `m_speed_est_fast`
- `m_phase_now`
- `mcpwm_foc_init()`
- `mcpwm_foc_reset()`
- `mcpwm_foc_control()`

Removed from the rewritten controller files:

- `rtU`, `rtY`, `rtP`, `rtDW`
- `ExtU_*`, `ExtY_*`, `DW_*`, `P_*`, `RT_MODEL`
- MATLAB generated integer aliases such as `int16_T`, `uint16_T`, `boolean_T`

The FOC path is organized as:

1. electrical phase / hall or encoder update
2. current Clarke transform
3. Park transform
4. current filtering
5. `id/iq` targets
6. d/q PI current control
7. voltage limiting
8. inverse Park
9. SVPWM common-mode injection
10. phase duty output

The 16-kHz path remains fixed-point/Q15 to avoid introducing `sinf/cosf` floating-point work on STM32F103.

## `bldc.c` preservation rule

The hardware ISR architecture remains in `bldc.c`:

- `DMA1_Channel1_IRQHandler` remains the motor-control ISR.
- ADC/DMA buffer usage remains in the same interrupt path.
- Current-offset calibration stays in the current firmware style.
- Current reconstruction, MOE current chopping, timer CCR updates and open-loop SVPWM remain in `bldc.c`.
- Telemetry-only Clarke/Park remains measurement-only and does not alter COM/SINE/SVPWM control laws.

Only the old generated-controller input/output objects were replaced by the VESC-style `m_motor_1/m_motor_2` interface, plus variable naming cleanup required by that interface.

## EEPROM compatibility

The parameter table remains at 61 persistent parameters with the same names and EEPROM indexes `0..60`. The existing EEPROM storage/scaling interface is retained.

## Telemetry compatibility

`SerialFeedback` keeps the same 35 packed fields in the same order, including the same command, rpm, dq, input-current, ADC, status, encoder and ISR timing fields. The USART3 telemetry framing was not redesigned.
