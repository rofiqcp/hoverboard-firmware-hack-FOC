#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PIO_HOME="${PLATFORMIO_CORE_DIR:-$HOME/.platformio}"
OPENOCD="$PIO_HOME/packages/tool-openocd/bin/openocd"
OCD_SCRIPTS="$PIO_HOME/packages/tool-openocd/openocd/scripts"
APP_ELF="$ROOT/.pio/build/APP_STLINK/firmware.elf"
BOOT_ELF="$ROOT/.pio/build/BOOTLOADER_STLINK/firmware.elf"

[[ -x "$OPENOCD" ]] || { echo "STLINK_INSTALL_FAIL: OpenOCD not found" >&2; exit 2; }
command -v pio >/dev/null || { echo "STLINK_INSTALL_FAIL: pio not found" >&2; exit 2; }
echo "[1/6] Build relocated application"
pio run -e APP_STLINK

echo "[2/6] Build resident bootloader"
pio run -e BOOTLOADER_STLINK

[[ -f "$APP_ELF" && -f "$BOOT_ELF" ]] || {
  echo "STLINK_INSTALL_FAIL: build products missing" >&2
  exit 2
}

mkdir -p "$ROOT/backups"
STAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP="$ROOT/backups/f103_full_before_bootloader_${STAMP}.bin"
OCD=("$OPENOCD" -s "$OCD_SCRIPTS" -f interface/stlink.cfg -c "adapter speed 100" -f target/stm32f1x.cfg)

echo "[3/6] Backup complete 256-KiB flash before any write"
"${OCD[@]}" -c "init; reset halt; dump_image {$BACKUP} 0x08000000 0x40000; shutdown"
SIZE="$(stat -c '%s' "$BACKUP")"
[[ "$SIZE" == "262144" ]] || {
  echo "STLINK_INSTALL_FAIL: backup size $SIZE, expected 262144" >&2
  exit 3
}
sha256sum "$BACKUP"

echo "[4/6] Program relocated application first and verify"
"${OCD[@]}" -c "program {$APP_ELF} verify; shutdown"
echo "[5/6] Program immutable bootloader last and verify"
"${OCD[@]}" -c "program {$BOOT_ELF} verify reset; shutdown"

echo "[6/6] Installation complete"
echo "STLINK_INSTALL_PASS"
echo "BACKUP=$BACKUP"
echo "Next: verify COMM_FW_VERSION over USART3/F411 before any motor command."
