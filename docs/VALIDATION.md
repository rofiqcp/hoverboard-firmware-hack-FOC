# Validation summary

## Completed checks

- Source/reference scan: no HD44780, PCF8574, I2C integration, USART2 integration, BOARD_VARIANT/BOARD 1 branch, non-USART input variants, or sideboard integration remains in `Src/`, `tools/`, or `platformio.ini`.
- `platformio.ini`: one environment only (`VARIANT_USART`), source directory explicitly `Src`, STM32F103xE/HAL/linker flags retained.
- Preprocessor nesting check: PASS for all `Src/*.c` and `Src/*.h`.
- C brace-balance check: PASS for all `Src/*.c`.
- Rough application dead-function reference audit: PASS; no ordinary application function was found with definition-only usage. Toolchain entry points such as `_write`, HAL callbacks and IRQ handlers are intentionally externally referenced.
- Python syntax (`py_compile`): PASS.
- Python binary protocol unit test: PASS for 8-byte command encoding/checksum and 30-byte telemetry decoding/checksum including `foc_isr_cycles`/`foc_isr_cycles_max`.

## Toolchain limitation in this sandbox

A real `pio run -e VARIANT_USART` could not be executed because PlatformIO and `arm-none-eabi-gcc` are not installed in the execution environment. Attempting to install PlatformIO also failed because outbound package resolution is unavailable. Therefore this checkpoint is **statically validated but not claimed as a successful target build**.

Run on a machine with PlatformIO:

```bash
pio run -e VARIANT_USART
```

Then flash with ST-Link:

```bash
pio run -e VARIANT_USART -t upload
```
