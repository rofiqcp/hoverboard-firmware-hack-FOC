#!/usr/bin/env python3
import argparse
import os
import subprocess
import sys
from pathlib import Path


def parse_int(value: str) -> int:
    return int(value, 0)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True)
    ap.add_argument("--address", required=True, type=parse_int)
    ap.add_argument("--max-size", required=True, type=parse_int)
    ap.add_argument("--adapter-khz", type=int, default=100)
    args = ap.parse_args()

    image = Path(args.image).resolve()
    if not image.is_file():
        raise SystemExit(f"STLINK_UPLOAD_FAIL: image missing: {image}")
    size = image.stat().st_size
    if size <= 0 or size > args.max_size:
        raise SystemExit(f"STLINK_UPLOAD_FAIL: size {size} exceeds {args.max_size}")

    pio_home = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio"))
    openocd = pio_home / "packages/tool-openocd/bin/openocd"
    scripts = pio_home / "packages/tool-openocd/openocd/scripts"
    if not openocd.is_file():
        raise SystemExit(f"STLINK_UPLOAD_FAIL: OpenOCD missing: {openocd}")

    cmd = [
        str(openocd), "-s", str(scripts), "-f", "interface/stlink.cfg",
        "-c", f"adapter speed {args.adapter_khz}", "-f", "target/stm32f1x.cfg",
        "-c", f"program {{{image}}} 0x{args.address:08X} verify reset; shutdown",
    ]
    print(f"[STLINK] image={image.name} size={size} address=0x{args.address:08X}")
    rc = subprocess.run(cmd, check=False).returncode
    if rc != 0:
        raise SystemExit(rc)
    print("STLINK_BIN_UPLOAD_PASS")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
