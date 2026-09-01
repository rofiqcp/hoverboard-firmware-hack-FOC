#!/usr/bin/env python3
"""VESC-standard USART3 client for the bare-metal STM32F103 dual hoverboard port.

Motor mapping:
  Left  = local VESC serial controller
  Right = virtual CAN controller ID 2 using COMM_FORWARD_CAN

The control worker refreshes setpoints at 50 Hz (VESC timeout is 500 ms) and
polls selective mc_values telemetry from both motors.
"""
from __future__ import annotations
import argparse
import struct
import threading
import time
from dataclasses import dataclass

try:
    import serial
except ImportError:
    serial = None

COMM_FW_VERSION = 0
COMM_GET_VALUES = 4
COMM_SET_DUTY = 5
COMM_SET_CURRENT = 6
COMM_SET_CURRENT_BRAKE = 7
COMM_SET_RPM = 8
COMM_ALIVE = 30
COMM_FORWARD_CAN = 34
COMM_GET_VALUES_SELECTIVE = 50
COMM_PING_CAN = 62
RIGHT_ID = 2
POLE_PAIRS = 15
STOP_ERPM = 5 * POLE_PAIRS  # 5 mechanical rpm

# currentMotor,currentIn,Id,Iq,duty,rpm,Vin,fault,vescId,Vd,Vq
VALUE_MASK = sum(1 << b for b in (2, 3, 4, 5, 6, 7, 8, 15, 17, 19, 20))


def crc16(data: bytes) -> int:
    crc = 0
    for x in data:
        crc ^= x << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(payload: bytes) -> bytes:
    n = len(payload)
    if not 0 < n <= 65535:
        raise ValueError("invalid VESC payload size")
    if n <= 255:
        head = bytes((2, n))
    else:
        head = bytes((3, (n >> 8) & 0xFF, n & 0xFF))
    c = crc16(payload)
    return head + payload + bytes((c >> 8, c & 0xFF, 3))


class PacketDecoder:
    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        out = []
        while self.buf:
            try:
                start = next(i for i, b in enumerate(self.buf) if b in (2, 3, 4))
            except StopIteration:
                self.buf.clear()
                break
            if start:
                del self.buf[:start]
            if not self.buf:
                break
            kind = self.buf[0]
            h = kind
            if len(self.buf) < h:
                break
            if kind == 2:
                n = self.buf[1]
            elif kind == 3:
                n = (self.buf[1] << 8) | self.buf[2]
                if n < 255:
                    del self.buf[0]
                    continue
            else:
                n = (self.buf[1] << 16) | (self.buf[2] << 8) | self.buf[3]
                if n < 65535:
                    del self.buf[0]
                    continue
            total = h + n + 3
            if len(self.buf) < total:
                break
            raw = bytes(self.buf[:total])
            del self.buf[:total]
            if raw[-1] != 3:
                continue
            payload = raw[h:h+n]
            rx_crc = (raw[h+n] << 8) | raw[h+n+1]
            if crc16(payload) == rx_crc:
                out.append(payload)
        return out


@dataclass
class Values:
    current_motor: float = 0.0
    current_in: float = 0.0
    id: float = 0.0
    iq: float = 0.0
    duty: float = 0.0
    rpm: float = 0.0
    vin: float = 0.0
    fault: int = 0
    vesc_id: int = 0
    vd: float = 0.0
    vq: float = 0.0

    def short(self) -> str:
        return (f"id={self.vesc_id} rpm={self.rpm:.0f} duty={100*self.duty:.1f}% "
                f"Imot={self.current_motor:.2f}A Iin={self.current_in:.2f}A "
                f"Id={self.id:.2f}A Iq={self.iq:.2f}A Vd={self.vd:.2f}V "
                f"Vq={self.vq:.2f}V Vin={self.vin:.1f}V fault={self.fault}")


def _i16(data: bytes, i: int, scale: float):
    return struct.unpack_from(">h", data, i)[0] / scale, i + 2


def _i32(data: bytes, i: int, scale: float):
    return struct.unpack_from(">i", data, i)[0] / scale, i + 4


def parse_selective(payload: bytes, expected_mask: int = VALUE_MASK) -> Values:
    if len(payload) < 5 or payload[0] != COMM_GET_VALUES_SELECTIVE:
        raise ValueError("not COMM_GET_VALUES_SELECTIVE")
    mask = struct.unpack_from(">I", payload, 1)[0]
    if mask != expected_mask:
        raise ValueError(f"mask mismatch 0x{mask:08x}")
    i = 5
    v = Values()
    for bit in range(22):
        if not (mask & (1 << bit)):
            continue
        if bit == 0: _, i = _i16(payload, i, 10)
        elif bit == 1: _, i = _i16(payload, i, 10)
        elif bit == 2: v.current_motor, i = _i32(payload, i, 100)
        elif bit == 3: v.current_in, i = _i32(payload, i, 100)
        elif bit == 4: v.id, i = _i32(payload, i, 100)
        elif bit == 5: v.iq, i = _i32(payload, i, 100)
        elif bit == 6: v.duty, i = _i16(payload, i, 1000)
        elif bit == 7: v.rpm, i = _i32(payload, i, 1)
        elif bit == 8: v.vin, i = _i16(payload, i, 10)
        elif bit in (9, 10, 11, 12): _, i = _i32(payload, i, 10000)
        elif bit in (13, 14): _, i = _i32(payload, i, 1)
        elif bit == 15: v.fault, i = payload[i], i + 1
        elif bit == 16: _, i = _i32(payload, i, 1_000_000)
        elif bit == 17: v.vesc_id, i = payload[i], i + 1
        elif bit == 18: i += 6
        elif bit == 19: v.vd, i = _i32(payload, i, 1000)
        elif bit == 20: v.vq, i = _i32(payload, i, 1000)
        elif bit == 21: i += 1
    return v


class VescDual:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.15):
        if serial is None:
            raise RuntimeError("pyserial required: python -m pip install pyserial")
        self.ser = serial.Serial(port, baud, timeout=0.01)
        self.timeout = timeout
        self.dec = PacketDecoder()
        self.io_lock = threading.Lock()

    def close(self):
        self.ser.close()

    def send(self, payload: bytes):
        self.ser.write(frame(payload))

    def recv(self, expected_cmd: int, timeout: float | None = None) -> bytes:
        end = time.monotonic() + (self.timeout if timeout is None else timeout)
        while time.monotonic() < end:
            chunk = self.ser.read(256)
            if not chunk:
                continue
            for p in self.dec.feed(chunk):
                if p and p[0] == expected_cmd:
                    return p
        raise TimeoutError(f"no reply for COMM {expected_cmd}")

    def transact(self, payload: bytes, expected_cmd: int, timeout: float | None = None) -> bytes:
        with self.io_lock:
            self.send(payload)
            return self.recv(expected_cmd, timeout)

    @staticmethod
    def fwd(payload: bytes) -> bytes:
        return bytes((COMM_FORWARD_CAN, RIGHT_ID)) + payload

    def ping_can(self):
        p = self.transact(bytes((COMM_PING_CAN,)), COMM_PING_CAN, 0.5)
        return list(p[1:])

    def fw(self, right=False) -> bytes:
        req = bytes((COMM_FW_VERSION,))
        return self.transact(self.fwd(req) if right else req, COMM_FW_VERSION, 0.5)

    def set_current(self, left: float, right: float):
        l = bytes((COMM_SET_CURRENT,)) + struct.pack(">i", round(left * 1000))
        r = bytes((COMM_SET_CURRENT,)) + struct.pack(">i", round(right * 1000))
        with self.io_lock:
            self.send(l); self.send(self.fwd(r))

    def set_rpm(self, left: int, right: int):
        l = bytes((COMM_SET_RPM,)) + struct.pack(">i", int(left))
        r = bytes((COMM_SET_RPM,)) + struct.pack(">i", int(right))
        with self.io_lock:
            self.send(l); self.send(self.fwd(r))

    def set_duty(self, left: float, right: float):
        l = bytes((COMM_SET_DUTY,)) + struct.pack(">i", round(left * 100000))
        r = bytes((COMM_SET_DUTY,)) + struct.pack(">i", round(right * 100000))
        with self.io_lock:
            self.send(l); self.send(self.fwd(r))

    def brake(self, left_a: float, right_a: float):
        l = bytes((COMM_SET_CURRENT_BRAKE,)) + struct.pack(">i", round(abs(left_a) * 1000))
        r = bytes((COMM_SET_CURRENT_BRAKE,)) + struct.pack(">i", round(abs(right_a) * 1000))
        with self.io_lock:
            self.send(l); self.send(self.fwd(r))

    def values(self, right=False) -> Values:
        req = bytes((COMM_GET_VALUES_SELECTIVE,)) + struct.pack(">I", VALUE_MASK)
        p = self.transact(self.fwd(req) if right else req, COMM_GET_VALUES_SELECTIVE)
        return parse_selective(p)


class ReplWorker:
    def __init__(self, link: VescDual, hz: float, telemetry_hz: float):
        self.link = link
        self.period = 1.0 / hz
        self.telemetry_period = 1.0 / telemetry_hz
        self.lock = threading.Lock()
        self.mode = "current"
        self.left = 0.0
        self.right = 0.0
        self.active = False
        self.stop_flag = False
        self.exit = False
        self.last_l = Values(vesc_id=1)
        self.last_r = Values(vesc_id=2)
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def set(self, mode: str, left: float, right: float):
        with self.lock:
            self.mode, self.left, self.right = mode, left, right
            self.active, self.stop_flag = True, False

    def stop_controlled(self):
        with self.lock:
            self.stop_flag = True
            self.active = True

    def release(self):
        self.link.set_current(0.0, 0.0)
        with self.lock:
            self.active = False
            self.stop_flag = False

    def run(self):
        next_tick = next_tel = time.monotonic()
        while not self.exit:
            now = time.monotonic()
            if now < next_tick:
                time.sleep(min(0.002, next_tick - now)); continue
            next_tick += self.period
            with self.lock:
                mode, l, r, active, stopping = self.mode, self.left, self.right, self.active, self.stop_flag
            try:
                if active:
                    if stopping:
                        self.link.brake(1.2, 1.2)
                    elif mode == "current": self.link.set_current(l, r)
                    elif mode == "rpm": self.link.set_rpm(int(l), int(r))
                    elif mode == "duty": self.link.set_duty(l, r)
                if now >= next_tel:
                    next_tel = now + self.telemetry_period
                    self.last_l = self.link.values(False)
                    self.last_r = self.link.values(True)
                    if stopping and abs(self.last_l.rpm) <= STOP_ERPM and abs(self.last_r.rpm) <= STOP_ERPM:
                        self.link.set_current(0.0, 0.0)
                        with self.lock:
                            self.active = False; self.stop_flag = False
            except Exception as e:
                print(f"[WARN] {e}")

    def shutdown(self):
        self.exit = True
        self.thread.join(timeout=1.0)
        try: self.link.set_current(0.0, 0.0)
        except Exception: pass


def parse_fw(p: bytes) -> str:
    if len(p) < 4: return repr(p)
    major, minor = p[1], p[2]
    end = p.find(b"\0", 3)
    hw = p[3:end].decode(errors="replace") if end >= 0 else "?"
    return f"FW {major}.{minor:02d} HW={hw}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--command-hz", type=float, default=50.0)
    ap.add_argument("--telemetry-hz", type=float, default=20.0,
                    help="selective VESC telemetry polling; 20 default, 50 supported for testing")
    args = ap.parse_args()
    link = VescDual(args.port, args.baud)
    try:
        print("local:", parse_fw(link.fw(False)))
        print("virtual CAN:", link.ping_can())
        print("right:", parse_fw(link.fw(True)))
        w = ReplWorker(link, args.command_hz, args.telemetry_hz)
        print("commands: current L R [A] | rpm L R [ERPM] | duty L R [-1..1] | stop | release | values | scan | fw | quit")
        while True:
            try: line = input("vesc-dual> ").strip()
            except (EOFError, KeyboardInterrupt): break
            if not line: continue
            a = line.split(); cmd = a[0].lower()
            try:
                if cmd == "current" and len(a) == 3: w.set("current", float(a[1]), float(a[2]))
                elif cmd == "rpm" and len(a) == 3: w.set("rpm", float(a[1]), float(a[2]))
                elif cmd == "duty" and len(a) == 3: w.set("duty", float(a[1]), float(a[2]))
                elif cmd == "stop": w.stop_controlled()
                elif cmd == "release": w.release()
                elif cmd == "values": print("L", w.last_l.short()); print("R", w.last_r.short())
                elif cmd == "scan": print("virtual CAN IDs:", link.ping_can())
                elif cmd == "fw": print("L", parse_fw(link.fw(False))); print("R", parse_fw(link.fw(True)))
                elif cmd in ("quit", "exit", "q"): break
                else: print("usage: current L R | rpm L R | duty L R | stop | release | values | scan | fw | quit")
            except Exception as e:
                print("[ERR]", e)
        w.shutdown()
    finally:
        link.close()

if __name__ == "__main__":
    main()
