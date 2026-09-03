# Firmware Tools

Folder ini dipisahkan antara tool operasional, regression host, pengujian hardware, support files, dan hasil test.

## Struktur

```text
tools/
├── vesc_debug.py        # CLI utama untuk diagnosis VESC/FOC
├── vesc_dual.py         # protocol VESC 6.00 + virtual CAN motor ID 2
├── hoverserial.py       # protocol serial hoverboard/legacy
├── run_all_checks.py    # semua regression non-aktuatif
├── requirements.txt
├── tests/
│   ├── host/            # aman dijalankan tanpa motor
│   └── hardware/        # dapat menggerakkan motor; gunakan dengan guard
├── support/             # HAL stubs untuk host compilation
└── results/             # CSV/bin/json hasil pengujian
```

## Mulai dari sini

Full regression tanpa aktuasi motor:

```bash
python3 tools/run_all_checks.py
```

Diagnosis board melalui USART3:

```bash
python3 tools/vesc_debug.py /dev/ttyUSB0 diag --motor both
```

Realtime VESC Tool-compatible telemetry check:

```bash
python3 tools/tests/hardware/test_vesc_tool_rt50.py /dev/ttyUSB0 --seconds 5 --hz 50
```

Hall detection repeat test dan duty/speed test berada di `tests/hardware/`; jalankan hanya saat roda aman untuk berputar.

## Aturan Folder

- `tests/host/`: deterministic regression, simulator, serializer, EEPROM, FOC math, protocol contract.
- `tests/hardware/`: script yang membuka serial dan/atau dapat memberi setpoint motor.
- `support/`: hanya stub header untuk build host; bukan source firmware.
- `results/speed_pid/<timestamp>/`: satu run berisi `raw.csv`, `summary.csv`, `meta.json`, dan snapshot config bila ada.
- `results/response_lab/`: data eksperimen duty/response lama yang tetap dipertahankan.

## Alur Pengembangan

1. Edit firmware di `Src/`.
2. Jalankan `python3 tools/run_all_checks.py`.
3. Build PlatformIO.
4. Upload hanya setelah host regression PASS.
5. Mulai hardware test dari read-only diagnostic, lalu setpoint rendah dengan fault/current/ERPM guard.

Jangan menaruh file test baru di root `tools/`: host regression masuk `tests/host/`, sedangkan script yang menyentuh motor masuk `tests/hardware/`.
