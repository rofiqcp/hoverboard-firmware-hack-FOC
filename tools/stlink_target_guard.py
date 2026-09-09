#!/usr/bin/env python3
"""Fail-closed ST-Link target guard for the AGV STM32F103RCT6 motor controller."""
import argparse
import re
import subprocess
from pathlib import Path

EXPECTED_CORE = "Cortex-M3"
EXPECTED_DEV_ID = 0x414  # STM32F103 high-density (xC/xD/xE)


def verify_f103_target(openocd: Path, scripts: Path, speed: int = 100) -> str:
    cmd = [str(openocd), "-s", str(scripts), "-f", "interface/stlink.cfg",
           "-c", "transport select swd", "-f", "target/stm32f1x.cfg",
           "-c", f"adapter speed {int(speed)}",
           "-c", "init; flash info 0; reset run; shutdown"]
    cp = subprocess.run(cmd, text=True, stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT, check=False)
    out = cp.stdout or ""
    match = re.search(r"device id\s*=\s*0x([0-9a-fA-F]+)", out)
    raw_id = int(match.group(1), 16) if match else None
    dev_id = (raw_id & 0xFFF) if raw_id is not None else None
    core_ok = re.search(r"\bCortex-M3\b", out) is not None
    if cp.returncode != 0 or not core_ok or dev_id != EXPECTED_DEV_ID:
        core = EXPECTED_CORE if core_ok else "non-M3/unknown"
        did = f"0x{dev_id:03X}" if dev_id is not None else "unknown"
        raise RuntimeError(
            f"STLINK_TARGET_REJECTED: expected STM32F103RCT6 {EXPECTED_CORE} "
            f"DEV_ID=0x{EXPECTED_DEV_ID:03X}, got core={core} DEV_ID={did}; NO FLASH WRITE PERFORMED")
    print(f"STLINK_TARGET_F103_PASS core={EXPECTED_CORE} DEV_ID=0x{dev_id:03X}", flush=True)
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--openocd", required=True)
    ap.add_argument("--scripts", required=True)
    ap.add_argument("--speed", type=int, default=100)
    args = ap.parse_args()
    try:
        verify_f103_target(Path(args.openocd), Path(args.scripts), args.speed)
    except RuntimeError as exc:
        raise SystemExit(str(exc))


if __name__ == "__main__":
    main()
