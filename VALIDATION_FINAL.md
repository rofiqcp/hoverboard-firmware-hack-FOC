# Validation Final V10

Tree ini divalidasi dengan `python3 tools/run_all_checks.py` setelah semua perubahan terakhir.

PASS:
- GCC `-Wall -Wextra -Werror` untuk seluruh source aktif.
- Clang `-Wall -Wextra -Werror` untuk seluruh source aktif.
- Fixed-point FOC runtime test.
- Mode 1 permille voltage scaling dan mirror kanan.
- Mode 2 speed PI -> Vq, scheduler 1/3, ERPM conversion.
- Mode 3 50 cA=0.50 A dan 1500 cA=15 A, current slew, PI output, stop brake/neutral.
- Mode 4 Id=2 A, Iq=0, synthetic phase, 6 A clamp, voltage ceiling.
- Reverse Hall convention + interpolation low-speed disable.
- VESC FW 7.01 protocol host test.
- Virtual CAN scan ID 2 dan `COMM_FORWARD_CAN` right command.
- VESC config serializer sizes.
- Python VESC dual packet/CRC test.
- `datatypes.h` SHA256 reference match.
- Static audit: no port wrappers/generated controller/current_scale leftovers.

PlatformIO ARM target build tidak dijalankan di environment pembuat ZIP karena `pio`/`arm-none-eabi-gcc` tidak tersedia. `platformio.ini` tetap disiapkan untuk `pio run -e VARIANT_USART` dan project warnings diperlakukan sebagai errors tanpa membocorkan `-Werror` ke framework STM32Cube.
