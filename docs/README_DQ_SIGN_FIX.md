> **Current integration:** see `README_FULL_MODES_GUI.md`. This file documents an earlier intermediate stage and may contain older frame sizes/mode notes.

# dq current, right-motor sign, and volatile warning fix

## Right motor display convention

The right motor is physically mirrored. Firmware still drives it internally with `pwmr = -cmdR`, but user-facing telemetry is normalized so a positive command means the same vehicle direction on both wheels.

- `cmdL`, `cmdR`: already user-facing; no extra sign inversion.
- `rpmL`: `rtY_Left.n_mot`.
- `rpmR`: `-rtY_Right.n_mot`.
- `iqL`: left raw q-axis current converted to ampere.
- `iqR`: negative of the right raw q-axis current, so positive torque direction matches positive `cmdR`.
- `idL`, `idR`: **not** direction-inverted; d-axis current is the flux-axis component.
- `idcL`, `idcR`: not direction-inverted; sign represents DC-bus power/current direction, not wheel mounting.
- Hall and ADC values stay raw.

The existing internal average-speed formula remains `(rtY_Left.n_mot - rtY_Right.n_mot)/2`; it already accounts for the mirrored right motor.

## `rtY_Left.n_mot` versus `odom_l`

They are not the same quantity.

- `rtY_Left.n_mot` is the generated controller's signed motor-speed estimate in RPM. It is derived from Hall transition timing and exported after fixed-point scaling.
- `odom_l` is an incremental Hall-sector position accumulator. `up_or_down()` contributes approximately one sector step per Hall transition and the result is wrapped modulo 9000.

They share Hall sensors as a source, but one is velocity and the other is accumulated position.

## Correct dq scaling

The generated controller does:

```c
rtb = rtU->i_phaXX << 4;
```

before Clarke/Park, and exports `rtY.iq` / `rtY.id` without a final `>>4`. Therefore `iq/id` are **Q4 current counts**:

```text
1 A = 16 * A2BIT_CONV dq units
```

With `A2BIT_CONV = 50`:

```text
1 A = 800 dq units
I[A] = dq_q4 / 800
```

The previous telemetry conversion divided only by 50, so displayed `iq/id` were 16 times too large. A displayed 3.2 A could therefore correspond to about 0.2 A after the correction.

Small nonzero `iq/id` at `cmd=0,0` can still be real measurement noise/residual current because ADC offset error, switching/dead-time, Hall-angle quantization, and the generated dq low-pass filter remain active while the controller is enabled. The telemetry intentionally does not add a cosmetic deadband.

## dq in every mode

Modes 1..3 copy the generated FOC outputs directly:

```c
foc_iqL_q4 = rtY_Left.iq;
foc_idL_q4 = rtY_Left.id;
foc_iqR_q4 = rtY_Right.iq;
foc_idR_q4 = rtY_Right.id;
```

Mode 4 bypasses the generated controller, so a measurement-only Clarke/Park calculation is added using the same raw phase-current inputs and the open-loop SVPWM phase. This keeps `iq/id` populated in telemetry.

Important: without Hall, encoder, or a rotor observer in mode 4, these values are **commanded-stator-frame dq**, not true rotor dq. They are useful for logging and approximate comparison, but they are not an exact torque/flux decomposition if the rotor slips relative to the commanded field.

## `volatile` warning

`FOC_ISR_CYC` and `FOC_ISR_MAX` are written by the ISR and therefore correctly remain `volatile`. The debug parameter table now stores `volatile void *`, and all generic debug reads/writes cast to `volatile <type> *`. This removes `-Wdiscarded-qualifiers` without stripping `volatile` or hiding the warning with an unsafe cast.
