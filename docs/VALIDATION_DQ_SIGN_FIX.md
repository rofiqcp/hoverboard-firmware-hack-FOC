# Validation: dq/sign/volatile update

Validated statically in the available sandbox:

- Python `py_compile`: PASS.
- Q4 current conversion check: PASS (`1600 q4 -> 2.00 A` with `A2BIT_CONV=50`).
- `rpmR` telemetry normalization: present (`-rtY_Right.n_mot`).
- `iqR` telemetry normalization: present (`-foc_iqR_q4`).
- `idR` intentionally not inverted.
- Modes 1..3 copy generated `rtY.iq/id` into telemetry snapshots.
- Mode 4 computes measurement-only dq from raw phase currents and the previous applied SVPWM phase.
- `parameter_entry.valueL/valueR` preserve `volatile` qualifier.
- Generic debug table reads/writes use `volatile` typed dereferences.
- Existing binary telemetry packet layout is unchanged, so the current `tools/hoverserial.py` protocol size/checksum remains compatible.

Not claimed: full `pio run` / ARM link PASS. The current execution environment does not contain the project ARM PlatformIO toolchain.
