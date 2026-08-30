# Build Fix 01

Fixes applied after `pio run -e two_hall` compiler output:

1. `Src/comms.c`
   - Restored the 10-entry debug protocol `errors[]` table.
   - Made it `static const char *const`.
   - Added bounds checking in `printError()` to prevent out-of-range array access.

2. `Src/eeprom.h`
   - Restored `EEPROM_START_ADDRESS` as `0x0803F000`.
   - `PAGE0_BASE_ADDRESS` now derives from `EEPROM_START_ADDRESS`.
   - Page0/Page1 remain contiguous 2 KiB pages at 0x0803F000 and 0x0803F800.

Static checks:
- Highest persistent parameter EEPROM address: 60.
- NB_OF_VAR: 64.
- Python host tools: `python -m py_compile tools/*.py` PASS.

PlatformIO/arm-none-eabi-gcc is not installed in this sandbox, so the target firmware build must still be verified on the user's PlatformIO environment.
