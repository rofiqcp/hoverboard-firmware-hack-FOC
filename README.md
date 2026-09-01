# Hoverboard STM32F103RCT6 — VESC Dual FOC V10 Final

Target hardware: STM32F103RCT6 hoverboard/EFeru dual motor, bare-metal STM32Cube, PWM/ADC 16 kHz, USART3 PB10/PB11 115200.

## Struktur
- `Src/motor/mcpwm_foc.c` — ISR-facing dual-motor state dan seluruh algoritma mode motor.
- `Src/motor/foc_math.c` — fixed-point Clarke/Park, PI, vector limit, centered SVPWM.
- `Src/motor/mc_interface.c` — interface VESC ke motor local/right.
- `Src/vesc/*` — `datatypes.h`, framing, CRC, config serializer, VESC protocol/virtual CAN.
- LEFT = local VESC; RIGHT = virtual CAN ID 2.

Tidak ada `port_*`, `BLDC_controller*`, `bldc.c`, `rtwtypes.h`, atau `current_scale.h`.

## Satuan dan algoritma mode legacy

| Mode | Command terminal | Algoritma |
|---|---|---|
| 1 VLT | permille (`100=10%`, `1000=100%`) | Hall electrical angle + direct `Vq`, `Vd=0`, centered SVPWM |
| 2 SPD | mechanical RPM (`50=50 rpm`) | Hall estimator + speed PI -> `Vq`, `Id=0`; PI update 16 kHz/3 = 5.333 kHz |
| 3 TRQ | centiampere (`50=0.50 A`, `1500=15 A`) | Hall FOC + `Iq` current PI, `Id=0`, current slew + voltage-circle anti-windup |
| 4 sensorless | ampere Id (`2=2 A`) | synthetic electrical phase, `Id` PI, `Iq=0`, current feedback tetap ADC; Hall hanya telemetry |

### Current scale
`A2BIT_CONV=50 count/A`, current FOC Q4 sehingga `1 A = 50*16 = 800 Q4`.

### Mode 4 safety
Default 10 mechanical rpm, acceleration 20 rpm/s, alignment 600 ms, Id slew 4 A/s, Id command clamp 6 A, phase/DC fast trip 8 A.

## Satuan VESC Tool / Python standar
- `COMM_SET_DUTY`: ratio -1..1.
- `COMM_SET_CURRENT`: ampere.
- `COMM_SET_RPM`: ERPM. Dengan 15 pole-pair, 750 ERPM = 50 mechanical RPM.
- `mc_values.rpm`: ERPM.
- Motor kanan dinormalisasi di protocol boundary sehingga command positif berarti arah kendaraan yang sama dengan motor kiri.

## Compile
```bash
pio run -e VARIANT_USART
```
`-Wall -Wextra -Werror` hanya diterapkan ke source project melalui `build_src_flags`, bukan STM32Cube HAL bawaan.

## Host regression
```bash
python3 tools/run_all_checks.py
```
Target:
```text
ALL_FINAL_HOST_CHECKS_PASS
```

## Hardware test awal
Roda diangkat terlebih dahulu.

```text
mode 1
start 30 30
drive 50 50
stop

mode 2
start 20 20
drive 50 50
stop

mode 3
start 25 25
drive 50 50
drive 100 100
stop

mode 4
SET SVPWM_RPM 10
start 1 1
drive 2 2
stop
```

Mode 3 adalah torque/current control, jadi pada roda bebas arus positif akan terus mempercepat motor. Gunakan mode 2 bila yang ingin dipertahankan adalah RPM.
