# USART3 six-mode firmware + terminal + GUI

This package extends the cleaned BOARD-0 / USART3-only firmware with a single runtime mode selector and one common 50-Hz telemetry format for comparison testing.

## Runtime modes

| Mode | Control algorithm | Host command meaning | dq telemetry |
|---|---|---|---|
| 1 | FOC VLT | voltage command | generated FOC `iq/id` |
| 2 | FOC SPD | speed target | generated FOC `iq/id` |
| 3 | FOC TRQ | torque/current target | generated FOC `iq/id` |
| 4 | open-loop sensorless SVPWM | electrical frequency/modulation command | measurement-only dq in commanded stator frame |
| 5 | generated 6-step commutation (COM) | voltage command | measurement-only dq using generated electrical angle |
| 6 | generated sine PWM (SIN) | voltage command | measurement-only dq using generated electrical angle |

Mode changes are accepted only after the independent Left/Right commands have ramped fully to `cmdL=0, cmdR=0`.

Important comparison note: the same numeric command does **not** represent the same physical setpoint in every mode. Modes 1, 5 and 6 are voltage-like, mode 2 is speed, mode 3 is torque/current, and mode 4 is open-loop electrical frequency/modulation.

## Common telemetry — 50 Hz

The packed USART3 feedback frame is 58 bytes and contains, for every mode:

- applied rate-limited `cmdL`, `cmdR`
- normalized host-direction `rpmL`, `rpmR`
- `iqL`, `iqR`, `idL`, `idR` in centiampere on the wire (Python shows ampere)
- `idcL`, `idcR` in centiampere
- battery voltage and board temperature
- Hall patterns Left/Right
- raw ADC `dcl, rla, rlb, dcr, rrb, rrc`
- enabled/timeout/fault/calibration status
- current runtime mode
- calibration progress
- 50-Hz telemetry sequence number
- `foc_isr_cycles`
- XOR checksum

`telemetry_seq` advances on every scheduled 50-Hz slot. The Python logger writes every valid frame and reports skipped sequence slots rather than silently hiding them.

### dq meaning

Modes 1–3 use `rtY_Left/Right.iq` and `.id` from the generated Clarke/Park FOC path. The generated code shifts phase-current input by 4, therefore conversion to ampere is:

`I[A] = dq_q4 / (16 * A2BIT_CONV)`

With `A2BIT_CONV=50`, 1 A = 800 dq counts.

Mode 4 bypasses the generated controller. Its dq values are measurement-only and referenced to the commanded open-loop stator electrical frame; they are not guaranteed to equal true rotor torque/flux currents if the rotor slips. Hall is read passively for RPM telemetry only and is not used to generate SVPWM.

Modes 5–6 run the generated COM/SIN algorithms. Their comparison dq is calculated from the same measured phase currents using the generated controller's `a_elecAngle`; it does not feed back into COM/SIN control.

The physically mirrored Right motor is internally driven with `pwmr=-cmdR`. Host telemetry normalizes `rpmR` and `iqR`, so positive means the same forward/torque direction as Left. `idR`, raw ADC, Hall and DC-link current are not direction-flipped.

## ADC current-offset calibration

Calibration is automatic at **every boot**. The first `ADC_CALIBRATION_SAMPLES=2000` motor-ISR samples are used to establish offsets for:

`rlA, rlB, rrB, rrC, dcl, dcr`

At 16 kHz this is approximately 125 ms of sampling. PWM outputs are not enabled during calibration.

Manual calibration is also available through the same USART3 interface:

`CALIBRATE`

It is accepted only at full STOP. `tools/gui.py` exposes this as **CALIBRATE ADC** with a confirmation dialog and progress indicator.

## Interactive terminal

Install dependencies:

```bash
python3 -m pip install -r tools/requirements.txt
```

Run:

```bash
python3 tools/hoverserial.py --port /dev/ttyUSB0
```

Example:

```text
hover> mode 1
hover> start 50,50
hover> drive 100,80
hover> stop
hover> mode 5
hover> start 30,30
hover> stop
```

`start` opens a CSV, every valid firmware telemetry frame is saved at 50 Hz, and `stop` sends target `0,0` while preserving the firmware rate limiter. The CSV is closed automatically only after three consecutive telemetry frames confirm applied `cmd=0,0`.

The terminal display itself defaults to 10 Hz for readability but logging remains 50 Hz. The fixed-width line keeps columns aligned while values change.

## GUI

Run:

```bash
python3 tools/gui.py
```

The GUI enables Qt high-DPI behavior, reads the primary screen's available pixel geometry, sizes itself to about 92% × 90% of that area, and centers automatically.

Features:

- serial port discovery and USART connection
- all six runtime modes
- independent `cmdL` / `cmdR`
- START + CSV, DRIVE, smooth STOP + automatic save
- manual current-ADC calibration
- Left/Right RPM, Iq, Id, Idc, Hall
- all six raw ADC channels
- battery, temperature, ISR cycles/us/load, status, sequence and measured telemetry rate
- rolling RPM, Iq/Id, DC-current and ISR graphs
- raw fixed-width telemetry view
- firmware debug console on the same USART3

## Safety

Start low, especially in mode 4 open-loop SVPWM. Mode 4 can lose rotor synchronism because it has no rotor observer/feedback for control. Mode changes and manual calibration are intentionally locked until rate-limited commands have reached zero.
