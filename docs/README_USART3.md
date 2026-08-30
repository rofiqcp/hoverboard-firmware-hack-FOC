> **Current integration:** see `README_FULL_MODES_GUI.md`. This file documents an earlier intermediate stage and may contain older frame sizes/mode notes.

# USART3-only hoverboard firmware profile

This source tree is intentionally specialized for the default hoverboard **board 0** hardware mapping and a single communication interface: **USART3 on PB10/PB11 at 115200 baud**.

Removed integrations: HD44780, PCF8574, I2C UI, USART2 control/debug, ADC/PPM/PWM/Nunchuk/iBUS input variants, sideboards, Hovercar/Hoverboard/TranspOtter/Skateboard variant branches, and BOARD 1 pin mapping.

## USART3 multiplexing

USART3 carries three logical streams:

1. Host -> controller binary drive frame: `<HhhH` little-endian = `0xABCD, steer, speed, checksum`.
2. Controller -> host binary telemetry frame: 30 bytes (`<HhhhhhhhhHIIH`).
3. Host/controller ASCII debug protocol: `GET`, `SET`, `WATCH`, `INIT`, `SAVE`, `HELP`.

The RX parser separates binary drive frames from ASCII debug lines on the same DMA-backed USART3 stream. Invalid/missing drive frames trigger the serial timeout and request `OPEN_MODE`.

## FOC ISR monitoring

`DMA1_Channel1_IRQHandler()` measures execution time with Cortex-M3 DWT CYCCNT:

- `foc_isr_cycles`: most recent ISR duration in CPU cycles.
- `foc_isr_cycles_max`: maximum observed duration since boot.

At 64 MHz CPU and 16 kHz FOC, one ISR period is **4000 CPU cycles = 62.5 us**. Both counters are included in binary telemetry and exposed in the ASCII debug protocol as `FOC_ISR_CYC` and `FOC_ISR_MAX`.

## Python host tool

Install dependency:

```bash
python -m pip install -r tools/requirements.txt
```

List ports:

```bash
python tools/hoverserial.py --list-ports
```

Interactive control + debug + telemetry:

```bash
python tools/hoverserial.py --port COM3
# Linux example: --port /dev/ttyUSB0
```

Monitor only (does not send drive commands):

```bash
python tools/hoverserial.py --port COM3 --monitor
```

Interactive examples:

```text
drive 0 200
stop
get FOC_ISR_CYC
watch FOC_ISR_CYC
watch FOC_ISR_MAX
get BATV
set I_MOT_MAX 10
save
```

The Python tool sends three zero-command frames on exit as a safety stop request.
