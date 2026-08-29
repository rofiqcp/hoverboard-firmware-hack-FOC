#!/usr/bin/env python3
"""USART3 host tool for the cleaned hoverboard firmware.

One port carries:
- binary drive commands (host -> STM32)
- binary telemetry (STM32 -> host)
- ASCII debug protocol (GET/SET/WATCH/INIT/SAVE/HELP)
"""
from __future__ import annotations

import argparse
import struct
import sys
import threading
import time
from dataclasses import dataclass

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit("pyserial is required: python -m pip install pyserial") from exc

START = 0xABCD
START_BYTES = struct.pack("<H", START)
CMD = struct.Struct("<HhhH")
FB = struct.Struct("<HhhhhhhhhHIIH")
CPU_HZ = 64_000_000
FOC_HZ = 16_000
FOC_BUDGET_CYCLES = CPU_HZ // FOC_HZ


def xor16(words):
    value = 0
    for word in words:
        value ^= int(word) & 0xFFFF
    return value & 0xFFFF


def make_command(steer: int, speed: int) -> bytes:
    steer = max(-1000, min(1000, int(steer)))
    speed = max(-1000, min(1000, int(speed)))
    return CMD.pack(START, steer, speed, xor16((START, steer, speed)))


@dataclass
class Telemetry:
    cmd1: int
    cmd2: int
    speed_r: int
    speed_l: int
    wheel_r: int
    wheel_l: int
    battery_x100: int
    temp_x10: int
    status: int
    foc_cycles: int
    foc_cycles_max: int

    @property
    def enabled(self): return bool(self.status & 0x01)
    @property
    def timeout(self): return bool(self.status & 0x02)
    @property
    def left_fault(self): return bool(self.status & 0x04)
    @property
    def right_fault(self): return bool(self.status & 0x08)

    def summary(self) -> str:
        us = self.foc_cycles * 1_000_000 / CPU_HZ
        load = self.foc_cycles * 100 / FOC_BUDGET_CYCLES
        max_us = self.foc_cycles_max * 1_000_000 / CPU_HZ
        flags = []
        if self.enabled: flags.append("ENA")
        if self.timeout: flags.append("TIMEOUT")
        if self.left_fault: flags.append("LFAULT")
        if self.right_fault: flags.append("RFAULT")
        return (f"cmd=({self.cmd1:+5d},{self.cmd2:+5d}) rpm=(L{self.speed_l:+5d},R{self.speed_r:+5d}) "
                f"V={self.battery_x100/100:5.2f} T={self.temp_x10/10:5.1f}C "
                f"foc_isr_cycles={self.foc_cycles:4d} ({us:6.2f}us,{load:5.1f}%) "
                f"max={self.foc_cycles_max:4d} ({max_us:6.2f}us) status={','.join(flags) or 'IDLE'}")


def decode_feedback(packet: bytes) -> Telemetry | None:
    if len(packet) != FB.size:
        return None
    fields = FB.unpack(packet)
    if fields[0] != START:
        return None
    expected = xor16(fields[:10] + (fields[10] & 0xFFFF, fields[10] >> 16,
                                    fields[11] & 0xFFFF, fields[11] >> 16))
    if fields[12] != expected:
        return None
    return Telemetry(*fields[1:12])


class HoverSerial:
    def __init__(self, port: str, baud: int):
        self.ser = serial.Serial(port, baud, timeout=0.05, write_timeout=0.5)
        self.stop_event = threading.Event()
        self.buffer = bytearray()
        self.write_lock = threading.Lock()
        self.latest: Telemetry | None = None
        self.reader = threading.Thread(target=self._reader_loop, daemon=True)
        self.last_print = 0.0

    def start(self):
        self.ser.reset_input_buffer()
        self.reader.start()

    def close(self):
        self.stop()
        self.stop_event.set()
        self.reader.join(timeout=0.5)
        self.ser.close()

    def send_drive(self, steer: int, speed: int):
        with self.write_lock:
            self.ser.write(make_command(steer, speed))

    def stop(self):
        if self.ser.is_open:
            for _ in range(3):
                try:
                    self.send_drive(0, 0)
                    time.sleep(0.02)
                except serial.SerialException:
                    break

    def debug(self, command: str):
        command = command.strip()
        if command:
            with self.write_lock:
                self.ser.write((command + "\n").encode("ascii", "strict"))

    def _print_ascii(self, data: bytes):
        if not data:
            return
        text = data.decode("ascii", "ignore")
        for line in text.replace("\r", "\n").split("\n"):
            line = line.strip()
            if line:
                print(f"[DBG] {line}")

    def _consume(self):
        while self.buffer:
            idx = self.buffer.find(START_BYTES)
            if idx < 0:
                # Keep one byte in case it is the first byte of the start marker.
                if len(self.buffer) > 1:
                    self._print_ascii(bytes(self.buffer[:-1]))
                    del self.buffer[:-1]
                return
            if idx > 0:
                self._print_ascii(bytes(self.buffer[:idx]))
                del self.buffer[:idx]
            if len(self.buffer) < FB.size:
                return
            packet = bytes(self.buffer[:FB.size])
            telemetry = decode_feedback(packet)
            if telemetry is None:
                self._print_ascii(bytes(self.buffer[:1]))
                del self.buffer[:1]
                continue
            del self.buffer[:FB.size]
            self.latest = telemetry
            now = time.monotonic()
            if now - self.last_print >= 0.10:
                print("[TEL]", telemetry.summary())
                self.last_print = now

    def _reader_loop(self):
        while not self.stop_event.is_set():
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException as exc:
                print(f"[ERR] serial read: {exc}", file=sys.stderr)
                self.stop_event.set()
                return
            if chunk:
                self.buffer.extend(chunk)
                self._consume()


def print_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    for p in ports:
        print(f"{p.device:12s} {p.description} {p.hwid}")


def interactive(link: HoverSerial, steer: int, speed: int, rate_hz: float):
    lock = threading.Lock()
    state = [steer, speed]
    active = True

    def tx_loop():
        period = 1.0 / max(1.0, rate_hz)
        while active and not link.stop_event.is_set():
            with lock:
                s, v = state
            try:
                link.send_drive(s, v)
            except serial.SerialException as exc:
                print(f"[ERR] serial write: {exc}", file=sys.stderr)
                return
            time.sleep(period)

    tx = threading.Thread(target=tx_loop, daemon=True)
    tx.start()
    print("Commands: drive <steer> <speed> | stop | get [name] | set <name> <value> | watch <name> | save | init <name> | help [name] | quit")
    try:
        while not link.stop_event.is_set():
            line = input("hover> ").strip()
            if not line:
                continue
            parts = line.split()
            op = parts[0].lower()
            if op == "drive" and len(parts) == 3:
                s = max(-1000, min(1000, int(parts[1])))
                v = max(-1000, min(1000, int(parts[2])))
                with lock:
                    state[:] = [s, v]
            elif op == "stop":
                with lock:
                    state[:] = [0, 0]
                link.stop()
            elif op in {"get", "set", "watch", "save", "init", "help"}:
                link.debug(line.upper())
            elif op in {"quit", "exit", "q"}:
                break
            else:
                print("Unknown command or arguments.")
    finally:
        active = False
        with lock:
            state[:] = [0, 0]
        link.stop()
        tx.join(timeout=0.3)


def main():
    ap = argparse.ArgumentParser(description="Hoverboard USART3 control/debug/telemetry tool")
    ap.add_argument("--port", default="/dev/ttyUSB0", help="Serial port, e.g. COM3 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--list-ports", action="store_true")
    ap.add_argument("--steer", type=int, default=0, help="Initial steer command [-1000,1000]")
    ap.add_argument("--speed", type=int, default=0, help="Initial speed command [-1000,1000]")
    ap.add_argument("--rate", type=float, default=20.0, help="Drive command rate Hz")
    ap.add_argument("--monitor", action="store_true", help="Telemetry/debug monitor only; does not send drive frames")
    args = ap.parse_args()

    if args.list_ports:
        print_ports()
        if not args.port:
            return
    if not args.port:
        ap.error("--port is required (or use --list-ports)")

    link = HoverSerial(args.port, args.baud)
    link.start()
    print(f"Connected {args.port} @ {args.baud}; feedback={FB.size} bytes; FOC budget={FOC_BUDGET_CYCLES} cycles")
    try:
        if args.monitor:
            while not link.stop_event.is_set():
                time.sleep(0.2)
        else:
            interactive(link, args.steer, args.speed, args.rate)
    except KeyboardInterrupt:
        pass
    finally:
        link.close()


if __name__ == "__main__":
    main()
