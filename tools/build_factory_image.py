#!/usr/bin/env python3
from pathlib import Path
import hashlib
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
BOOT = ROOT / ".pio/build/BOOTLOADER_STLINK/firmware.bin"
APP = ROOT / ".pio/build/APP_STLINK/firmware.bin"
OUT = ROOT / "dist/f103_factory_boot_app.bin"

FLASH_BASE = 0x08000000
BOOT_SIZE = 0x2800
APP_BASE = FLASH_BASE + BOOT_SIZE
APP_REGION_SIZE = 0x1E000
RAM_LO = 0x20000000
RAM_HI = 0x2000C000
def fail(msg: str) -> None:
    raise SystemExit(f"FACTORY_IMAGE_FAIL: {msg}")


def main() -> None:
    if not BOOT.is_file() or not APP.is_file():
        fail("build BOOTLOADER_STLINK and APP_STLINK first")
    boot = BOOT.read_bytes()
    app = APP.read_bytes()
    if not boot or len(boot) > BOOT_SIZE:
        fail(f"bootloader size {len(boot)} exceeds {BOOT_SIZE}")
    if not app or len(app) > APP_REGION_SIZE:
        fail(f"application size {len(app)} exceeds {APP_REGION_SIZE}")
    if len(app) < 8:
        fail("application vector table missing")

    sp, rv = struct.unpack_from("<II", app, 0)
    if not (RAM_LO <= sp <= RAM_HI and (sp & 3) == 0):
        fail(f"invalid application MSP 0x{sp:08X}")
    pc = rv & ~1
    if (rv & 1) == 0 or not (APP_BASE <= pc < APP_BASE + APP_REGION_SIZE):
        fail(f"invalid application Reset_Handler 0x{rv:08X}")

    image = bytearray(b"\xFF" * (BOOT_SIZE + len(app)))
    image[: len(boot)] = boot
    image[BOOT_SIZE : BOOT_SIZE + len(app)] = app
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_bytes(image)

    digest = hashlib.sha256(image).hexdigest()
    print(
        "FACTORY_IMAGE_PASS "
        f"boot={len(boot)} app={len(app)} total={len(image)} "
        f"app_base=0x{APP_BASE:08X} sha256={digest}"
    )
    print(f"FACTORY_IMAGE={OUT}")


if __name__ == "__main__":
    main()
