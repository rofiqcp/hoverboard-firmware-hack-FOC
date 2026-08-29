#!/usr/bin/env python3
"""Shared USART3 protocol, telemetry and CSV utilities for hoverboard tools."""
from __future__ import annotations

import csv
import struct
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit("Install serial support: python3 -m pip install -r tools/requirements.txt") from exc

START = 0xABCD
START_BYTES = struct.pack("<H", START)
CMD = struct.Struct("<HhhH")
# Src/main.c SerialFeedback (packed), 58 bytes:
# H + 12h + 11H + II + H
FB = struct.Struct("<H12h11HIIH")

CPU_HZ = 64_000_000
FOC_HZ = 16_000
TELEMETRY_HZ = 50
FOC_BUDGET_CYCLES = CPU_HZ // FOC_HZ
CMD_MIN = -1000
CMD_MAX = 1000

MODE_NAMES = {
    1: "FOC VLT",
    2: "FOC SPD",
    3: "FOC TRQ",
    4: "SVPWM sensorless",
    5: "6-step commutation",
    6: "Sine PWM",
}

DQ_REFERENCES = {
    1: "generated rotor dq",
    2: "generated rotor dq",
    3: "generated rotor dq",
    4: "commanded stator dq (measurement only)",
    5: "generated electrical-angle dq (measurement only)",
    6: "generated electrical-angle dq (measurement only)",
}


def clamp_cmd(value: int) -> int:
    return max(CMD_MIN, min(CMD_MAX, int(value)))


def xor16(words) -> int:
    value = 0
    for word in words:
        value ^= int(word) & 0xFFFF
    return value & 0xFFFF


def make_command(cmd_l: int, cmd_r: int) -> bytes:
    cmd_l = clamp_cmd(cmd_l)
    cmd_r = clamp_cmd(cmd_r)
    return CMD.pack(START, cmd_l, cmd_r, xor16((START, cmd_l, cmd_r)))


def available_ports() -> list[str]:
    return [p.device for p in list_ports.comports()]


def _feedback_checksum(packet_without_checksum: bytes) -> int:
    words = struct.unpack("<" + "H" * (len(packet_without_checksum) // 2), packet_without_checksum)
    return xor16(words)


@dataclass(frozen=True)
class Telemetry:
    cmd_l: int
    cmd_r: int
    rpm_l: int
    rpm_r: int
    iq_l_cA: int
    iq_r_cA: int
    id_l_cA: int
    id_r_cA: int
    idc_l_cA: int
    idc_r_cA: int
    battery_x100: int
    temp_x10: int
    hall_l: int
    hall_r: int
    adc_dcl: int
    adc_rla: int
    adc_rlb: int
    adc_dcr: int
    adc_rrb: int
    adc_rrc: int
    status: int
    mode: int
    calibration_permille: int
    telemetry_seq: int
    foc_cycles: int

    @property
    def enabled(self) -> bool: return bool(self.status & 0x01)
    @property
    def timeout(self) -> bool: return bool(self.status & 0x02)
    @property
    def left_fault(self) -> bool: return bool(self.status & 0x04)
    @property
    def right_fault(self) -> bool: return bool(self.status & 0x08)
    @property
    def calibrating(self) -> bool: return bool(self.status & 0x10)
    @property
    def stopped(self) -> bool: return self.cmd_l == 0 and self.cmd_r == 0
    @property
    def battery_v(self) -> float: return self.battery_x100 / 100.0
    @property
    def temperature_c(self) -> float: return self.temp_x10 / 10.0
    @property
    def iq_l_a(self) -> float: return self.iq_l_cA / 100.0
    @property
    def iq_r_a(self) -> float: return self.iq_r_cA / 100.0
    @property
    def id_l_a(self) -> float: return self.id_l_cA / 100.0
    @property
    def id_r_a(self) -> float: return self.id_r_cA / 100.0
    @property
    def idc_l_a(self) -> float: return self.idc_l_cA / 100.0
    @property
    def idc_r_a(self) -> float: return self.idc_r_cA / 100.0
    @property
    def foc_us(self) -> float: return self.foc_cycles * 1_000_000.0 / CPU_HZ
    @property
    def foc_load_pct(self) -> float: return self.foc_cycles * 100.0 / FOC_BUDGET_CYCLES
    @property
    def mode_name(self) -> str: return MODE_NAMES.get(self.mode, f"MODE {self.mode}")
    @property
    def dq_reference(self) -> str: return DQ_REFERENCES.get(self.mode, "unknown")

    @property
    def status_text(self) -> str:
        flags = ["ENA" if self.enabled else "DIS"]
        if self.timeout: flags.append("TIMEOUT")
        if self.left_fault: flags.append("FAULT-L")
        if self.right_fault: flags.append("FAULT-R")
        if self.calibrating: flags.append("CAL")
        return "/".join(flags)

    def summary(self) -> str:
        """Fixed width output: columns stay aligned while values change."""
        return (
            f"M{self.mode:d} "
            f"cmd={self.cmd_l:+5d},{self.cmd_r:+5d}  "
            f"rpm={self.rpm_l:+5d},{self.rpm_r:+5d}  "
            f"iq={self.iq_l_a:+7.2f},{self.iq_r_a:+7.2f}A  "
            f"id={self.id_l_a:+7.2f},{self.id_r_a:+7.2f}A  "
            f"idc={self.idc_l_a:+6.2f},{self.idc_r_a:+6.2f}A  "
            f"hall={self.hall_l & 7:03b},{self.hall_r & 7:03b}  "
            f"adc={self.adc_dcl:4d},{self.adc_rla:4d},{self.adc_rlb:4d},"
            f"{self.adc_dcr:4d},{self.adc_rrb:4d},{self.adc_rrc:4d}  "
            f"V={self.battery_v:5.2f}  T={self.temperature_c:5.1f}C  "
            f"cyc={self.foc_cycles:4d}"
        )


def decode_feedback(packet: bytes) -> Telemetry | None:
    if len(packet) != FB.size or struct.unpack_from("<H", packet, 0)[0] != START:
        return None
    if _feedback_checksum(packet[:-2]) != struct.unpack_from("<H", packet, len(packet) - 2)[0]:
        return None
    v = FB.unpack(packet)
    return Telemetry(
        cmd_l=v[1], cmd_r=v[2], rpm_l=v[3], rpm_r=v[4],
        iq_l_cA=v[5], iq_r_cA=v[6], id_l_cA=v[7], id_r_cA=v[8],
        idc_l_cA=v[9], idc_r_cA=v[10], battery_x100=v[11], temp_x10=v[12],
        hall_l=v[13], hall_r=v[14], adc_dcl=v[15], adc_rla=v[16],
        adc_rlb=v[17], adc_dcr=v[18], adc_rrb=v[19], adc_rrc=v[20],
        status=v[21], mode=v[22], calibration_permille=v[23],
        telemetry_seq=v[24], foc_cycles=v[25],
    )


class CsvLogger:
    """Write every valid firmware frame; firmware telemetry is exactly 50 Hz nominal."""

    FIELDNAMES = [
        "host_time", "sample_index", "firmware_seq", "firmware_time_s",
        "mode", "mode_name", "dq_reference", "cmdL", "cmdR", "rpmL", "rpmR",
        "iqL_A", "iqR_A", "idL_A", "idR_A", "idcL_A", "idcR_A",
        "hallL", "hallR", "dcl_raw", "rla_raw", "rlb_raw", "dcr_raw", "rrb_raw", "rrc_raw",
        "battery_V", "temperature_C", "foc_isr_cycles", "foc_isr_us", "foc_isr_load_pct",
        "enabled", "timeout", "left_fault", "right_fault", "calibrating", "calibration_pct",
    ]

    def __init__(self, directory: Path):
        self.directory = Path(directory)
        self._lock = threading.Lock()
        self._fp = None
        self._writer = None
        self._path: Path | None = None
        self._rows = 0
        self._first_seq: int | None = None
        self._last_seq: int | None = None
        self._missed = 0

    @property
    def active(self) -> bool:
        with self._lock: return self._fp is not None
    @property
    def path(self) -> Path | None:
        with self._lock: return self._path
    @property
    def rows(self) -> int:
        with self._lock: return self._rows
    @property
    def missed(self) -> int:
        with self._lock: return self._missed

    def start(self, mode: int | None = None) -> Path:
        with self._lock:
            if self._fp is not None:
                return self._path  # type: ignore[return-value]
            self.directory.mkdir(parents=True, exist_ok=True)
            suffix = f"_mode{mode}" if mode in MODE_NAMES else ""
            self._path = self.directory / f"hover_{datetime.now():%Y%m%d_%H%M%S}{suffix}.csv"
            self._fp = self._path.open("w", newline="", encoding="utf-8")
            self._writer = csv.DictWriter(self._fp, fieldnames=self.FIELDNAMES)
            self._writer.writeheader()
            self._rows = 0
            self._first_seq = None
            self._last_seq = None
            self._missed = 0
            return self._path

    def write(self, t: Telemetry) -> None:
        with self._lock:
            if self._fp is None or self._writer is None:
                return
            if self._first_seq is None:
                self._first_seq = t.telemetry_seq
            if self._last_seq is not None:
                delta = (t.telemetry_seq - self._last_seq) & 0xFFFFFFFF
                if delta > 1:
                    self._missed += delta - 1
            self._last_seq = t.telemetry_seq
            fw_t = ((t.telemetry_seq - self._first_seq) & 0xFFFFFFFF) / TELEMETRY_HZ
            self._writer.writerow({
                "host_time": datetime.now().isoformat(timespec="milliseconds"),
                "sample_index": self._rows, "firmware_seq": t.telemetry_seq,
                "firmware_time_s": f"{fw_t:.6f}", "mode": t.mode,
                "mode_name": t.mode_name, "dq_reference": t.dq_reference,
                "cmdL": t.cmd_l, "cmdR": t.cmd_r, "rpmL": t.rpm_l, "rpmR": t.rpm_r,
                "iqL_A": f"{t.iq_l_a:.4f}", "iqR_A": f"{t.iq_r_a:.4f}",
                "idL_A": f"{t.id_l_a:.4f}", "idR_A": f"{t.id_r_a:.4f}",
                "idcL_A": f"{t.idc_l_a:.4f}", "idcR_A": f"{t.idc_r_a:.4f}",
                "hallL": f"{t.hall_l & 7:03b}", "hallR": f"{t.hall_r & 7:03b}",
                "dcl_raw": t.adc_dcl, "rla_raw": t.adc_rla, "rlb_raw": t.adc_rlb,
                "dcr_raw": t.adc_dcr, "rrb_raw": t.adc_rrb, "rrc_raw": t.adc_rrc,
                "battery_V": f"{t.battery_v:.3f}", "temperature_C": f"{t.temperature_c:.2f}",
                "foc_isr_cycles": t.foc_cycles, "foc_isr_us": f"{t.foc_us:.4f}",
                "foc_isr_load_pct": f"{t.foc_load_pct:.3f}", "enabled": int(t.enabled),
                "timeout": int(t.timeout), "left_fault": int(t.left_fault),
                "right_fault": int(t.right_fault), "calibrating": int(t.calibrating),
                "calibration_pct": f"{t.calibration_permille / 10.0:.1f}",
            })
            self._rows += 1
            if self._rows % TELEMETRY_HZ == 0:
                self._fp.flush()

    def stop(self) -> tuple[Path | None, int, int]:
        with self._lock:
            path, rows, missed = self._path, self._rows, self._missed
            if self._fp is not None:
                self._fp.flush(); self._fp.close()
            self._fp = None; self._writer = None
            return path, rows, missed


class HoverLink:
    def __init__(self, port: str, baud: int = 115200):
        self.port, self.baud = port, baud
        self.serial = serial.Serial(port, baudrate=baud, timeout=0.05, write_timeout=0.2)
        self._write_lock = threading.Lock()
        self._run = threading.Event(); self._run.set()
        self._thread = threading.Thread(target=self._reader, name="hover-usart3-rx", daemon=True)
        self.on_telemetry: Callable[[Telemetry], None] | None = None
        self.on_debug: Callable[[str], None] | None = None
        self.on_error: Callable[[str], None] | None = None
        self.latest: Telemetry | None = None
        self._buffer = bytearray()
        self._text = bytearray()
        self._thread.start()

    def send_command(self, cmd_l: int, cmd_r: int) -> None:
        payload = make_command(cmd_l, cmd_r)
        with self._write_lock:
            if self.serial.is_open: self.serial.write(payload)

    def send_debug(self, line: str) -> None:
        data = line.strip().encode("ascii", errors="ignore") + b"\r\n"
        with self._write_lock:
            if self.serial.is_open: self.serial.write(data)

    def close(self) -> None:
        self._run.clear()
        try: self._thread.join(timeout=0.5)
        except RuntimeError: pass
        if self.serial.is_open: self.serial.close()

    def _emit_text(self, data: bytes) -> None:
        if not data: return
        self._text.extend(data)
        while True:
            pos_n = self._text.find(b"\n")
            if pos_n < 0: break
            raw = bytes(self._text[:pos_n + 1]); del self._text[:pos_n + 1]
            line = raw.decode("utf-8", errors="replace").strip("\r\n\x00")
            if line and self.on_debug: self.on_debug(line)
        if len(self._text) > 1024:
            raw = bytes(self._text); self._text.clear()
            line = raw.decode("utf-8", errors="replace").strip("\r\n\x00")
            if line and self.on_debug: self.on_debug(line)

    def _reader(self) -> None:
        try:
            while self._run.is_set():
                data = self.serial.read(max(1, self.serial.in_waiting or 1))
                if not data: continue
                self._buffer.extend(data)
                while self._buffer:
                    idx = self._buffer.find(START_BYTES)
                    if idx < 0:
                        keep = 1 if self._buffer[-1:] == START_BYTES[:1] else 0
                        cut = len(self._buffer) - keep
                        self._emit_text(bytes(self._buffer[:cut])); del self._buffer[:cut]
                        break
                    if idx > 0:
                        self._emit_text(bytes(self._buffer[:idx])); del self._buffer[:idx]
                        continue
                    if len(self._buffer) < FB.size: break
                    packet = bytes(self._buffer[:FB.size])
                    t = decode_feedback(packet)
                    if t is None:
                        self._emit_text(bytes(self._buffer[:1])); del self._buffer[:1]
                        continue
                    del self._buffer[:FB.size]
                    self.latest = t
                    if self.on_telemetry: self.on_telemetry(t)
        except Exception as exc:
            if self._run.is_set() and self.on_error:
                self.on_error(str(exc))
