# STM32F103RCT6 VESC Bootloader and Upload Paths

The F103 uses USART3 on PB10/PB11 at 2000000 baud for both normal VESC traffic and firmware updates. No BOOT0 or NRST wire is required after the resident bootloader has been installed once with ST-Link.

## Flash layout

- `0x08000000..0x080027FF`: immutable recovery bootloader, 10 KiB
- `0x08002800..0x080207FF`: active application, 120 KiB
- `0x08020800..0x0803E7FF`: staged firmware image, 120 KiB
- `0x0803E800..0x0803EFFF`: update metadata, 2 KiB
- `0x0803F000..0x0803FFFF`: existing emulated EEPROM, 4 KiB

The updater follows the VESC `COMM_ERASE_NEW_APP`, `COMM_WRITE_NEW_APP_DATA`, and `COMM_JUMP_TO_BOOTLOADER` flow. A size+CRC16 header is staged before the image. The active app is not erased until the staged image validates.
## PlatformIO environments

Build/update the application with one of these environments:

```bash
pio run -e APP_STLINK
pio run -e APP_USART_PC
pio run -e APP_F411
```

Initial bootloader installation/recovery remains ST-Link only:

```bash
pio run -e BOOTLOADER_STLINK
```

For the first installation, use `tools/install_bootloader_stlink.sh`. It refuses to write until a complete 256-KiB backup succeeds, programs and verifies the relocated app first, then programs the immutable bootloader last and resets. Keep an ST-Link header available as the final recovery path.
### Direct USB-UART to F103

Connect TX/RX crossed to F103 USART3 (`PB10=TX`, `PB11=RX`) and common GND. Then run:

```bash
pio run -e APP_USART_PC -t upload --upload-port /dev/ttyUSBX
```

### Through the STM32F411 gateway

The F411 exposes F103 VESC packets through the localhost TCP proxy on port `65102`. With ROS/F411 bridge running:

```bash
pio run -e APP_F411 -t upload
```

The same TCP endpoint can be selected in VESC Tool. The web operator console remains on `http://localhost:5000`.
## Recovery and safety guarantees

During update the application releases both motors and clears both advanced-timer MOE bits before touching flash. The bootloader holds all six high-side gates low and all six active-low low-side gates high.

If power is lost while the bootloader is copying a valid staged image, the `PENDING` metadata is preserved. The next boot retries the copy from the still-intact staging region. If no valid application exists, the bootloader stays in recovery and accepts the same VESC firmware commands over USART3.

The bootloader itself is intentionally not self-updatable. Updating or recovering the bootloader requires ST-Link so a malformed network/serial firmware package cannot overwrite the last recovery path.
