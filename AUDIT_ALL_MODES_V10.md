# Audit Semua Mode V10

## Mode 1 — VLT
- Command legacy adalah permille.
- `100 -> Vq = 1440` dari ceiling 14400; `1000 -> Vq = 14400` sebelum PWM margin clamp.
- `Vd=0`.
- Hall menentukan electrical phase.
- Mode 1 sengaja tidak memakai current PI karena log hardware menunjukkan mode ini sudah bekerja stabil.

## Mode 2 — SPD
- Command legacy adalah mechanical RPM.
- Hall RPM dihitung dari period: `16000*60/(15*6*period) = 10667/period`.
- Invalid/non-adjacent Hall transitions ditolak.
- Interpolasi Hall memakai hysteresis 30 rpm ON / 15 rpm OFF dan reverse convention `pos+1-fraction`.
- Speed PI memakai error Q4 dan mengeluarkan `Vq` langsung, sama dengan arsitektur generated controller lama.
- Regulator update 1 dari 3 ADC frames = 5.333 kHz; PWM/current sensing tetap 16 kHz.
- Closed-loop vector ceiling 12800 menyisakan PWM/current-sampling headroom.

## Mode 3 — TRQ / Current
- Command adalah centiampere: 50=0.50 A; 100=1 A; 1500=15 A.
- Target menjadi `Iq_ref`, `Id_ref=0`.
- Current target dislew 10 A/s agar command tidak menghentak.
- Phase current ADC dikonversi fixed-point Q4 dan disaturasi ±27200 sebelum Clarke/Park.
- PI Id lalu voltage-circle menentukan headroom PI Iq; anti-windup melihat limit yang benar.
- Fast phase-current trip aktif pada 15 A; DC-link trip memakai I_DC_MAX 17 A.
- STOP pada RPM > deadband memberi brake current 1.20 A berlawanan arah; saat <=5 rpm kembali neutral duty 0.

## Mode 4 — Sensorless Id FOC
- Tidak memakai Hall/encoder sebagai angle control.
- Synthetic electrical phase internal; Hall tetap dibaca untuk telemetry.
- Command adalah Id dalam ampere: 2 -> Id target 2 A; Iq target selalu 0.
- Id target dislew 4 A/s.
- Alignment phase 3pi/2 selama 600 ms.
- Default 10 mechanical rpm, max 300 rpm; 15 pole-pair.
- Id safety clamp 6 A; phase/DC fast trip 8 A.
- Tidak ada offset +30 derajat tambahan pada synthetic phase.

## Standardisasi VESC
- VESC current = ampere.
- VESC duty = ratio -1..1.
- VESC RPM = ERPM; internal Hall estimator tetap mechanical RPM.
- 15 pole-pair: 750 ERPM = 50 mechanical RPM.
- Local=LEFT, virtual CAN ID 2=RIGHT.
- Right command/telemetry dinormalisasi pada protocol boundary: rpm, current, iq, duty, vq.

## Fixed-point / cadence
- ADC/PWM: 16 kHz.
- Control regulator cadence: 16 kHz / 3.
- Current: 800 Q4/A.
- Current LPF Q16 coefficient: 7864.
- Iq PI: Kp 1229 Q11, Ki 1229 Q16.
- Id PI: Kp 819 Q11, Ki 737 Q16.
- Speed PI: Kp 4833 Q11, Ki 251 Q16.
- Direct voltage max 14400; closed-loop max 12800.
