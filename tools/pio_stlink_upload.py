#!/usr/bin/env python3
import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

from stlink_target_guard import resolve_stlink_transport, verify_f103_target


def parse_int(value: str) -> int:
    return int(value, 0)


def unique_speeds(preferred: int):
    # Fast first when requested, then progressively more conservative SWD clocks.
    values = [preferred, 400, 100]
    out = []
    for value in values:
        value = max(50, min(int(value), 4000))
        if value not in out:
            out.append(value)
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True)
    ap.add_argument("--address", required=True, type=parse_int)
    ap.add_argument("--max-size", required=True, type=parse_int)
    ap.add_argument("--adapter-khz", type=int, default=1000)
    args = ap.parse_args()

    image = Path(args.image).resolve()
    if not image.is_file():
        raise SystemExit(f"STLINK_UPLOAD_FAIL: image missing: {image}")
    size = image.stat().st_size
    if size <= 0 or size > args.max_size:
        raise SystemExit(f"STLINK_UPLOAD_FAIL: size {size} exceeds {args.max_size}")

    pio_home = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio"))
    openocd_root = pio_home / "packages" / "tool-openocd"
    scripts = openocd_root / "openocd" / "scripts"
    openocd_candidates = [
        openocd_root / "bin" / "openocd.exe",
        openocd_root / "bin" / "openocd",
    ]
    openocd = next((candidate for candidate in openocd_candidates if candidate.is_file()), None)
    if openocd is None:
        checked = ", ".join(str(candidate) for candidate in openocd_candidates)
        raise SystemExit(f"STLINK_UPLOAD_FAIL: OpenOCD missing; checked: {checked}")
    if not scripts.is_dir():
        raise SystemExit(f"STLINK_UPLOAD_FAIL: OpenOCD scripts missing: {scripts}")

    transport = resolve_stlink_transport(scripts)

    try:
        verify_f103_target(openocd, scripts, 100)
    except RuntimeError as exc:
        raise SystemExit(str(exc))

    last_rc = 1
    speeds = unique_speeds(args.adapter_khz)
    for attempt, speed in enumerate(speeds, start=1):
        # target/stm32f1x.cfg sets 1000 kHz internally, so the explicit speed
        # must come AFTER the target file or it is silently overwritten.
        cmd = [
            str(openocd), "-s", str(scripts),
            "-f", "interface/stlink.cfg",
            "-c", f"transport select {transport}",
            "-f", "target/stm32f1x.cfg",
            "-c", f"adapter speed {speed}",
            # Use OpenOCD's program helper for binary images. It performs the
            # erase/program/verify sequence with the flash driver's supported
            # reset flow, which is more reliable with HLA ST-Link on Windows.
            "-c", (
                f"program {{{image}}} 0x{args.address:08X} verify reset exit"
            ),
        ]
        print(
            f"[STLINK] attempt={attempt}/{len(speeds)} image={image.name} "
            f"size={size} address=0x{args.address:08X} swd={speed}kHz",
            flush=True,
        )
        last_rc = subprocess.run(cmd, check=False).returncode
        if last_rc == 0:
            print(f"STLINK_BIN_UPLOAD_PASS swd={speed}kHz", flush=True)
            return
        if attempt < len(speeds):
            print(f"[STLINK] retrying at safer SWD clock after rc={last_rc}", file=sys.stderr, flush=True)
            time.sleep(0.30)
    raise SystemExit(last_rc)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
