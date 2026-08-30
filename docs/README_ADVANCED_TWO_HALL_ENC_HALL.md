# Hoverboard FOC v3 Advanced — two_hall / enc_hall

## Build profiles

```bash
pio run -e two_hall
pio run -e enc_hall
```

- `two_hall`: Left + Right use Hall feedback. Modes 1..6.
- `enc_hall`: Left uses AB quadrature encoder on TIM4 PB6/PB7 (x4, no index), Right remains Hall. Modes 1..7.
- USART3 PB10/PB11 @ 115200 is the only control/debug/telemetry interface.

## Modes

1. FOC voltage
2. Speed PID -> torque -> FOC current loops
3. FOC torque
4. Sensorless open-loop SVPWM
5. Six-step commutation
6. Sine PWM
7. Position PID -> speed PID -> torque -> FOC (`enc_hall`, Left only)

All modes publish the same 50-Hz comparison telemetry: cmdL/R, normalized RPM L/R, Iq/Id L/R, DC current L/R, Hall/synthetic Hall, six raw ADC channels, battery, temperature, ISR cycles/max, calibration state, and encoder/profile state.

## ADC current calibration

Boot and manual `CALIBRATE` use the same sequence:

1. Disable both bridges / MOE.
2. Settle for 1600 ISR samples = 100 ms at 16 kHz.
3. Accumulate exactly 2000 samples for rlA/rlB/rrB/rrC/dcl/dcr.
4. Use the arithmetic mean as the current offset.
5. Keep bridge disabled while main context clears the complete Left/Right generated-controller DWork/Input/Output state.
6. Re-run `BLDC_controller_initialize()` and reset outer PID/dq filters.

Manual calibration is accepted only after command ramp-down is complete and both motors are approximately stationary (`|rpm| <= 5`).

## EEPROM / RAM model

STM32F103RCT6 Flash is partitioned as:

- firmware: `0x08000000 .. 0x0803EFFF` (252 KiB)
- EEPROM page 0: `0x0803F000 .. 0x0803F7FF`
- EEPROM page 1: `0x0803F800 .. 0x0803FFFF`

At boot, persistent parameters are loaded from emulated EEPROM into RAM. `SET` changes RAM immediately and therefore affects a running controller immediately. EEPROM is changed only by `SAVE`.

SAVE is transactional: invalidate key -> write all persistent values -> write valid key last. The GUI first commands a smooth STOP and waits for `cmdL=cmdR=0` and `|rpmL/R|<=5` before sending SAVE.

Persistent tuning includes motor limits, independent q/d FOC PI coefficients, current filter/anti-windup limits, independent outer speed PID Kp/Ki/Kd, position PID/ranges, command rate/filter, encoder parameters/alignment, field weakening/phase advance, generated speed-loop internals, and commutation thresholds.

## enc_hall boot synchronization

`enc_hall` initializes TIM4 PB6/PB7 in encoder AB mode. Left Hall PB6/PB7 are not used as physical Hall inputs in this profile. Boot sequence after ADC calibration:

1. Record initial encoder position.
2. Drive Left only with low-command open-loop SVPWM sweep.
3. Determine encoder sign from observed movement.
4. Hold a fixed electrical stator vector and allow rotor settling.
5. Define encoder electrical offset, including compensation for the generated controller's internal `-30 degree` electrical-angle term.
6. Use external mechanical angle (`degree x16`) in the generated controller.
7. Position->speed->torque cascade returns Left to the position it had before the sweep.
8. Mark encoder synchronization ready. Normal motor enabling remains blocked if synchronization fails.

The sweep physically moves the Left motor during boot. First validation should be performed unloaded / wheels lifted and with current-limited power.

## Python tools

```bash
python3 -m pip install -r tools/requirements.txt
python3 tools/hoverserial.py --port /dev/ttyUSB0
python3 tools/gui.py
```

`gui.py` is PyQt5 + pyqtgraph and auto-sizes to the current display. It provides Dashboard, Graphs, RAM/EEPROM Tuning, and Raw/Console tabs. On every serial connection it sends `GET` and populates tuning widgets from the MCU's actual RAM `value/init/min/max`. Editing a widget sends `SET` to RAM with a short debounce. `SAVE EEPROM` performs smooth STOP before commit.

CSV records every valid firmware telemetry frame at the firmware's nominal 50 Hz. `firmware_seq` / `missed_seq` allow detection of dropped USART frames.
