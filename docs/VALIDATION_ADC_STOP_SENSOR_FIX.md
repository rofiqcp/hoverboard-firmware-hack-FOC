# ADC / STOP / SENSOR FIX — validation checkpoint

## Baseline used

The hardware-behavior baseline is the user-provided archive:
`hoverboard-firmware-hack-FOC yang sudah ok kalibrasi dan start stop nya.zip`.
The three user captures were also compared:

- `hover_20260830_180312_mode2.csv`
- `hover_20260830_180438_mode4.csv`
- `hover_20260830_180700 yang sudha ok kalibrasinya dan start stop nya.csv`

## What the captures prove

### Known-good capture
At STOP, the known-good capture stays at the current-sense midpoint:

- RLA ~1968
- RLB ~1952
- RRB ~1950
- RRC ~1936

and both Hall sensors traverse all six valid states during the run.

### Broken mode-2 capture before this patch
Before STOP, mode 2 is already around the expected midpoint (median RLA/RLB/RRB/RRC =
1969/1968/1955/1933). The acquisition timing is therefore not globally shifted.
Immediately after STOP the old VESC-style checkpoint changed `enabled` to 0 and phase
ADC channels floated, e.g. ~4018/4014 and later ~4001/317. This is a bridge-state
problem, not a calibration offset that should be numerically corrected.

### Broken mode-4 capture before this patch
Mode 4 shows median RLA/RLB/RRB/RRC ~2541/2535/2452/2437 while measured RPM remains
zero and dq telemetry reaches roughly +/-20 A. This indicates a stalled/high-current
open-loop state. The old hard phase-current chop cleared MOE, changing the current-sense
common-mode and making subsequent phase samples unreliable. The fix below does not force
real current samples back to 2000; it keeps the ADC operating point valid and current-limits
mode 4 with a synchronous zero vector.

## ADC timing / DMA audit

The actual ADC/timer sequence in this checkpoint is kept identical to the known-good
hardware baseline:

- PWM = 16 kHz center aligned.
- Right = TIM1, Left = TIM8.
- TIM8 TRGO UPDATE triggers ADC1 regular conversion.
- ADC1 + ADC2 = dual regular simultaneous.
- ADC clock = 16 MHz (PCLK2/4).
- phase-current channels = 7.5-cycle sampling.
- first DC-current rank = 1.5-cycle sampling, exactly as baseline.
- `LEFT_TIM->CNT = ADC_TOTAL_CONV_TIME = 80` timer ticks.
- DMA1 Channel1 = 5 x 32-bit dual-ADC words, circular.
- DMA buffer order remains exactly:
  `dcr,dcl,rlA,rlB,rrB,rrC,batt1,PA2,temp,PA3`.

`setup.c` now has `_Static_assert` checks for the complete DMA buffer byte offsets and for
`ADC_TOTAL_CONV_TIME == 80`. A future refactor that silently reorders the ADC DMA ABI or
changes this validated sampling offset will fail the target compile instead of silently
producing wrong channels.

## STOP fix

Normal STOP is now distinct from a hard safety shutdown:

1. Host zero is transmitted immediately by GUI/CLI.
2. Firmware zero command bypasses the command rate limiter and LPF immediately.
3. Outer speed/position state is reset.
4. Both motor targets are forced to zero.
5. FOC/SINE/COMM controllers return duty delta 0/0/0 and reset FOC target/integrators.
6. SVPWM command zero returns duty delta 0/0/0.
7. Normal STOP keeps MOE active at CCR=50/50/50: zero line-line voltage / synchronous zero vector.
8. MOE is cleared only for a true hard inhibit (timeout/fault/calibration/power state or DC-input overcurrent).

This restores the important behavior seen in the known-good closed-loop capture: STOP does
not leave the phase-current amplifiers floating. It also provides electrical zero-vector
braking instead of simply coasting with all MOSFET outputs disabled.

## Mode-4 phase-current limiting fix

For open-loop SVPWM only, phase-current over-limit no longer clears MOE. It now:

- forces U/V/W duty delta to 0/0/0 (50/50/50 actual CCR),
- keeps the bridge common-mode defined,
- does not advance the open-loop electrical phase while current is being chopped,
- resumes from the same electrical vector after current decays.

Hard DC-link overcurrent still clears MOE.

## Sensors in every mode

Sensor acquisition happens before mode dispatch on every motor ISR:

### `two_hall`
- Left physical Hall code: always sampled.
- Right physical Hall code: always sampled.
- Left Hall RPM: always updated.
- Right Hall RPM: always updated.
- ADC raw current/DC/battery/temp/PA2/PA3: independent of control mode.

### `enc_hall`
- Left TIM4 AB encoder position/RPM/electrical angle: available in every mode.
- Left `hallL` telemetry is a synthetic six-state Hall code derived from encoder angle because
  PB6/PB7 are used by encoder AB in this profile.
- Right physical Hall code/RPM: always sampled.
- ADC raw channels: independent of control mode.

Modes 1..7 no longer decide whether a physical speed sensor is acquired. In particular,
mode 4 no longer deliberately reports RPM=0 just because SVPWM control itself is open loop.
If a wheel physically rotates, its Hall/encoder speed remains visible.

## ADC validity status

The telemetry packet ABI and field order are unchanged. Extra status bits are used only
inside the existing 16-bit `status` field:

- bit 5: both phase-current ADC sides valid
- bit 6: left phase-current ADC valid
- bit 7: right phase-current ADC valid

The Python tools show `ADC-OK` or side validity and add these derived columns to new CSVs.
No telemetry field was inserted or reordered.

## Validation run in this environment

PASS with `gcc -std=gnu11 -Wall -Wextra -Werror -fsyntax-only` for both profiles on:

- `BLDC_controller.c`
- `BLDC_controller_data.c`
- `advanced_control.c`
- `bldc.c`
- `main.c`
- `util.c`
- `comms.c`

Host controller test PASS:

- FOC STOP -> zero duties + zero iq/id targets + PI reset
- SINE STOP -> zero duties
- COMMUTATION STOP -> zero duties
- controller disable -> zero duties/state

Static integration checks PASS:

- STOP bypasses smoothing
- STOP forces both motor targets zero
- neutral arm behavior restored
- phase current trip does not clear MOE
- left/right mode-4 soft chop uses zero vector
- Hall/RPM acquisition is mode independent
- SVPWM zero command returns zero vector

Python GUI/CLI/parser files pass `py_compile`.

`SerialFeedback` packed struct is byte-for-byte identical to the previous VESC-style
checkpoint (same ABI). EEPROM `PARAMETER` table count/index layout is not changed by this fix.

## Hardware acceptance test after flash

1. Boot with wheels free and wait for calibration completion.
2. At STOP after a valid serial connection, status should show `ADC-OK` and phase raw values
   should remain near their calibrated board-specific midpoints (for the supplied board,
   roughly 1.9k–2.0k). Do **not** require exactly 2000 on every board.
3. Mode 2: run, press STOP. Command must become 0 immediately; motor torque command must be zero;
   phase raw channels must not jump to ~0/~4k just because STOP was pressed.
4. Mode 4: start with a small command. Hall/encoder telemetry must still be live. If rotor is
   stalled, raw phase values may legitimately move away from 2k because real phase current is
   flowing; the soft current limiter should periodically apply zero vector instead of creating
   floating ADC samples.
5. Test modes 1,2,3,4,5,6 (and 7 for enc_hall): rotate the wheel and confirm Hall/RPM or encoder
   continues changing regardless of selected control algorithm.

Use `python3 tools/check_capture.py <new.csv>` to summarize valid ADC samples and sensor activity.
