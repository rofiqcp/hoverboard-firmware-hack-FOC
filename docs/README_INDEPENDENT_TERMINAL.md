> **Current integration:** see `README_FULL_MODES_GUI.md`. This file documents an earlier intermediate stage and may contain older frame sizes/mode notes.

# USART3 Independent Left/Right Terminal

This revision targets the cleaned **BOARD 0 / VARIANT_USART / USART3-only** firmware.

## Motor command meaning

The binary command frame is now explicitly:

- `cmdL`: Left motor command, `-1000..+1000`
- `cmdR`: Right motor command, `-1000..+1000`

There is no steering/speed mixer in this revision. Left and Right commands are independent.
The Right bridge keeps the firmware's physical-direction sign compensation internally (`pwmr = -cmdR`).

## Modes

- `mode 1` = VLT
- `mode 2` = SPD
- `mode 3` = TRQ
- `mode 4` = sensorless open-loop SVPWM

`CTRL_MOD` can only change when both requested commands are zero **and** the rate-limited applied `cmdL/cmdR` have reached exactly zero.

## Smooth STOP

`stop` does not zero PWM abruptly. The host continuously transmits target `0,0`, and firmware applies the existing `RATE` and low-pass `FILTER` during ramp-down. Only the final <=1 command-count residual is snapped to exact zero after the rate-limiter state itself has reached zero.

The feedback `cmd=` fields report the **applied rate-limited commands**, not merely the requested host target. This lets the host verify that ramp-down is actually complete.

## Telemetry

Binary feedback is 50 bytes and includes:

- applied `cmdL`, `cmdR`
- `rpmL`, `rpmR`
- Clarke/Park `iqL`, `iqR`, `idL`, `idR` in centiampere
- DC-link `idcL`, `idcR` in centiampere
- battery voltage and board temperature
- 3-bit Hall states for Left and Right
- raw ADC: `dcl`, `rla`, `rlb`, `dcr`, `rrb`, `rrc`
- status flags
- `foc_isr_cycles`

The Python terminal converts centiampere to ampere and prints compactly, for example:

```text
cmd=0,0 rpm=0,0 iq=2,3 id=0,0 idc=2,2 hall=001,011 dcl=1893 rla=1966 rlb=1954 dcr=1940 rrb=1944 rrc=1931 cycle=2876
```

### Current scaling

The FOC generated model's `rtY_Left/Right.iq` and `.id` values are in the same current-count scale as its phase-current inputs. With `A2BIT_CONV=50`, the conversion is:

```text
I[A] = current_counts / 50
```

The binary telemetry stores these currents as centiampere (`A * 100`). `idcL/idcR` are derived from the calibrated DCL/DCR current channels and are also sent as centiampere.

In mode 4 the generated FOC Clarke/Park block is bypassed, so its `iq/id` telemetry is reported as zero rather than claiming a rotor-referenced FOC current that was not calculated by the generated controller.

## Interactive Python terminal

Install dependencies:

```bash
python -m pip install -r tools/requirements.txt
```

Run, for example:

```bash
python tools/hoverserial.py --port /dev/ttyUSB0
```

or on Windows:

```bash
python tools/hoverserial.py --port COM3
```

Normal workflow:

```text
hover> mode 1
hover> start 50,50
hover> drive 100,100
hover> drive 150,80
hover> stop
```

- `start L,R`: creates a timestamped CSV then starts independent Left/Right commands.
- `drive L,R`: changes the command while that run is active.
- `stop`: sends target `0,0`, waits for rate-limited applied `cmdL/cmdR` to become `0,0`, then automatically saves/closes the CSV.
- `mode N`: accepted only while fully stopped.
- `status`: prints the latest telemetry once.
- `get`, `set`, `watch`, `save`, `init`: still pass through to the firmware ASCII debug console on USART3.
- `quit`: performs the same gradual stop and saves any active CSV before exiting.

CSV files are stored by default in `tools/logs/`. Every valid telemetry frame is recorded; the terminal display itself defaults to 10 Hz for readability.

`prompt_toolkit.patch_stdout` is used so asynchronous telemetry/debug messages do not overwrite the `hover>` prompt or partially typed text.
