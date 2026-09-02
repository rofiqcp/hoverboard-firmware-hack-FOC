# VALIDATION V16 — VESC 6.00 Dual Hall + RT 50 Hz

## Fokus revisi

- Nama VESC Tool: `motor_left` (local) dan `motor_right` (ID 2).
- Hall estimator memakai midpoint sector-center + interpolation edge-to-edge + phase rate-limit.
- Hall detect: current ramp 1000 x 1 ms, 3 sweep maju + 3 sweep mundur, 360 langkah/sweep, 5 ms/langkah.
- Current telemetry di-zero-kan saat bridge/control OFF.
- USART/VESC RX pending diubah dari single-slot menjadi FIFO 4 packet.
- Realtime VESC `GET_VALUES` dan `GET_VALUES_SETUP`/selective mendukung reply langsung dan periodik 20 ms (50 Hz).
- Hall detect timeout Python 20 s.

## Hasil host regression

```text
HOST_COMPILE_CHANGED_SOURCES_PASS
FOC_GCC_CLANG_RUNTIME_PASS
MOTOR_CONTROL_V12_GCC_CLANG_PASS
MOTOR_CONTROL_V13_GCC_CLANG_PASS
VESC_PROTOCOL_GCC_CLANG_PASS
CONFIG_SERIALIZER_GCC_CLANG_PASS
PY_VESC_DUAL_PACKET_PASS
V13_FEATURE_STATIC_PASS
V14_FEATURE_STATIC_PASS
V15_FEATURE_STATIC_PASS
V16_FEATURE_STATIC_PASS
VESC_DEBUG_SELFTEST_PASS
HALL_DETECT_ALGORITHM_GCC_CLANG_PASS
EEPROM_DUAL_PERSISTENCE_GCC_CLANG_PASS
STATIC_PROJECT_AUDIT_PASS
ALL_FINAL_HOST_CHECKS_PASS
```

Build source aktif diuji dengan GCC dan Clang memakai `-Wall -Wextra -Werror`.

## Catatan hardware

Environment validasi tidak memiliki `pio` maupun `arm-none-eabi-gcc`, sehingga V16 tidak mengklaim ARM/PlatformIO build PASS atau motor hardware PASS. Setelah flashing, ulangi Hall detect karena detector V16 lebih rapat dan estimator Hall berubah.
