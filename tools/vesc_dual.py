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
COMM_SET_POS = 9
COMM_SET_HANDBRAKE = 10
COMM_DETECT_HALL_FOC = 28
COMM_ALIVE = 30
COMM_FORWARD_CAN = 34
COMM_CUSTOM_APP_DATA = 36
COMM_GET_VALUES_SELECTIVE = 50
COMM_PING_CAN = 62
RIGHT_ID = 2
POLE_PAIRS = 15
STOP_ERPM = 5 * POLE_PAIRS  # 5 mechanical rpm

HB_MAGIC = b"HB"
HB_VERSION = 1
HB_GET_DIAG = 1
HB_GET_POS_STATE = 2
HB_SET_POS_LIMITS = 3
HB_SET_POS_TARGET = 4
HB_RESET_POSITION = 5

# currentMotor,currentIn,Id,Iq,duty,rpm,Vin,fault,vescId,Vd,Vq
VALUE_MASK = sum(1 << b for b in (2, 3, 4, 5, 6, 7, 8, 15, 16, 17, 19, 20))


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
    position: float = 0.0
    fault: int = 0
    vesc_id: int = 0
    vd: float = 0.0
    vq: float = 0.0

    def short(self) -> str:
        return (f"id={self.vesc_id} rpm={self.rpm:.0f} duty={100*self.duty:.1f}% "
                f"Imot={self.current_motor:.2f}A Iin={self.current_in:.2f}A "
                f"Id={self.id:.2f}A Iq={self.iq:.2f}A Vd={self.vd:.2f}V "
                f"Vq={self.vq:.2f}V Vin={self.vin:.1f}V pos={self.position:.2f}deg fault={self.fault}")



@dataclass
class PositionState:
    current: int
    target: int
    minimum: int
    maximum: int


@dataclass
class Diag:
    vesc_id: int
    control_mode: int
    state: int
    fault: int
    hall: int
    override: bool
    hall_store_ok: bool
    iq_target_a: float
    iq_ref_a: float
    iq_a: float
    id_a: float
    erpm: int
    duty: float
    position: int
    position_target: int
    position_min: int
    position_max: int
    hall_invalid: int
    current_trips: int
    rx_ok: int
    rx_crc_errors: int
    hall_table: list[int]
    hall_angle200: int | None = None
    hall_edge200: int | None = None
    hall_center200: int | None = None
    hall_direction: int | None = None
    hall_interp: bool | None = None
    hall_last_reject_reason: int | None = None
    hall_last_reject_from: int | None = None
    hall_last_reject_to: int | None = None
    phase_raw: int | None = None
    phase_hall_raw: int | None = None
    phase_target_raw: int | None = None
    hall_period: int | None = None
    hall_ticks: int | None = None
    hall_period_rejects: int | None = None
    hall_sequence_rejects: int | None = None
    current_offset_phase0: int | None = None
    current_offset_phase1: int | None = None
    current_offset_dc: int | None = None
    motor_poles: int | None = None
    pole_pairs: int | None = None
    gear_ratio: float | None = None
    motor_mech_rpm: float | None = None
    output_rpm: float | None = None
    rx_queue_drops: int | None = None
    foc_isr_cycles: int | None = None
    foc_isr_cycles_max: int | None = None
    phase_trip_count: int | None = None
    dc_trip_count: int | None = None
    phase_overcurrent_streak: int | None = None
    last_trip_source: int | None = None
    last_trip_phase0_a: float | None = None
    last_trip_phase1_a: float | None = None
    last_trip_phase2_a: float | None = None
    last_trip_dc_a: float | None = None
    last_trip_duty: float | None = None
    driven_offset0: int | None = None
    driven_offset1: int | None = None
    driven_offset_dc: int | None = None
    driven_offset_samples: int | None = None
    driven_offset_valid: bool | None = None
    driven_offset_calibrating: bool | None = None

    def short(self) -> str:
        return (
            f"id={self.vesc_id} mode={self.control_mode} state={self.state} fault={self.fault} "
            f"hall={self.hall} own={int(self.override)} Iq_tgt={self.iq_target_a:.3f}A "
            f"Iq_ref={self.iq_ref_a:.3f}A Iq={self.iq_a:.3f}A Id={self.id_a:.3f}A "
            f"erpm={self.erpm} duty={100*self.duty:.2f}% pos={self.position} "
            f"target={self.position_target} trips={self.current_trips} hall_bad={self.hall_invalid}"
        )


def parse_custom_header(payload: bytes, op: int) -> int:
    if len(payload) < 6 or payload[0] != COMM_CUSTOM_APP_DATA:
        raise ValueError("not COMM_CUSTOM_APP_DATA")
    if payload[1:3] != HB_MAGIC or payload[3] != HB_VERSION or payload[4] != op:
        raise ValueError("custom app header mismatch")
    return payload[5]


def parse_position_state(payload: bytes, op: int) -> PositionState:
    status = parse_custom_header(payload, op)
    if status:
        raise RuntimeError(f"custom position command failed status={status}")
    if len(payload) < 22:
        raise ValueError(f"short position state reply: {len(payload)}")
    return PositionState(*struct.unpack_from(">iiii", payload, 6))


def parse_diag(payload: bytes) -> Diag:
    status = parse_custom_header(payload, HB_GET_DIAG)
    if status:
        raise RuntimeError(f"diagnostic command failed status={status}")
    if len(payload) < 78:
        raise ValueError(f"short diagnostic reply: {len(payload)}")
    vid, mode, state, fault, hall, own, store_ok, _reserved = struct.unpack_from(">8B", payload, 6)
    vals = struct.unpack_from(">10i", payload, 14)
    hall_invalid, trips, rx_ok, rx_crc = struct.unpack_from(">4I", payload, 54)
    table = list(payload[70:78])
    ext = {}
    if len(payload) >= 104:
        (ang200, edge200, center200, direction_u8, interp, rej_reason, rej_from, rej_to) = struct.unpack_from(">8B", payload, 78)
        phase_raw, phase_hall_raw, phase_target_raw, hall_period, hall_ticks = struct.unpack_from(">5H", payload, 86)
        period_rej, sequence_rej = struct.unpack_from(">2I", payload, 96)
        direction = direction_u8 - 256 if direction_u8 >= 128 else direction_u8
        ext = dict(hall_angle200=ang200, hall_edge200=edge200, hall_center200=center200,
                   hall_direction=direction, hall_interp=bool(interp),
                   hall_last_reject_reason=rej_reason, hall_last_reject_from=rej_from,
                   hall_last_reject_to=rej_to, phase_raw=phase_raw,
                   phase_hall_raw=phase_hall_raw, phase_target_raw=phase_target_raw,
                   hall_period=hall_period, hall_ticks=hall_ticks,
                   hall_period_rejects=period_rej, hall_sequence_rejects=sequence_rej)
    if len(payload) >= 124:
        po0, po1, dco = struct.unpack_from(">3h", payload, 104)
        poles, pp = struct.unpack_from(">2B", payload, 110)
        gear_milli, mech_milli, out_milli = struct.unpack_from(">3i", payload, 112)
        ext.update(current_offset_phase0=po0, current_offset_phase1=po1, current_offset_dc=dco,
                   motor_poles=poles, pole_pairs=pp, gear_ratio=gear_milli/1000.0,
                   motor_mech_rpm=mech_milli/1000.0, output_rpm=out_milli/1000.0)
    if len(payload) >= 128:
        ext["rx_queue_drops"] = struct.unpack_from(">I", payload, 124)[0]
    if len(payload) >= 136:
        ext["foc_isr_cycles"], ext["foc_isr_cycles_max"] = struct.unpack_from(">2I", payload, 128)
    if len(payload) >= 156:
        phase_trips, dc_trips = struct.unpack_from(">2I", payload, 136)
        phase_streak, last_source = struct.unpack_from(">2B", payload, 144)
        lp0, lp1, lp2, ldc, lduty = struct.unpack_from(">5h", payload, 146)
        ext.update(phase_trip_count=phase_trips, dc_trip_count=dc_trips,
                   phase_overcurrent_streak=phase_streak, last_trip_source=last_source,
                   last_trip_phase0_a=lp0/50.0, last_trip_phase1_a=lp1/50.0,
                   last_trip_phase2_a=lp2/50.0, last_trip_dc_a=ldc/50.0,
                   last_trip_duty=lduty/1000.0)
    if len(payload) >= 166:
        do0, do1, dodc, dsamp, dvalid, dcal = struct.unpack_from(">3hH2B", payload, 156)
        ext.update(driven_offset0=do0, driven_offset1=do1, driven_offset_dc=dodc,
                   driven_offset_samples=dsamp, driven_offset_valid=bool(dvalid),
                   driven_offset_calibrating=bool(dcal))
    return Diag(
        vesc_id=vid, control_mode=mode, state=state, fault=fault, hall=hall,
        override=bool(own), hall_store_ok=bool(store_ok),
        iq_target_a=vals[0] / 1000.0, iq_ref_a=vals[1] / 1000.0,
        iq_a=vals[2] / 1000.0, id_a=vals[3] / 1000.0,
        erpm=vals[4], duty=vals[5] / 100000.0,
        position=vals[6], position_target=vals[7],
        position_min=vals[8], position_max=vals[9],
        hall_invalid=hall_invalid, current_trips=trips,
        rx_ok=rx_ok, rx_crc_errors=rx_crc, hall_table=table, **ext,
    )

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
        elif bit == 16: v.position, i = _i32(payload, i, 1000000)
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

    def set_pos(self, left_deg: float, right_deg: float):
        """Stock VESC single-turn position command, intentionally 0..360 deg.

        Use set_position_counts()/set_position_limits() for this project's signed
        int32 multi-turn Hall-count position API. Keeping the two APIs separate
        prevents VESC Tool's rotor-angle command from being reinterpreted as a
        long-range actuator position.
        """
        def enc(deg: float) -> bytes:
            if not 0.0 <= deg <= 360.0:
                raise ValueError("standard VESC position must be 0..360 degrees")
            return bytes((COMM_SET_POS,)) + struct.pack(">i", round(deg * 1000000.0))
        with self.io_lock:
            self.send(enc(left_deg)); self.send(self.fwd(enc(right_deg)))

    def brake(self, left_a: float, right_a: float):
        l = bytes((COMM_SET_CURRENT_BRAKE,)) + struct.pack(">i", round(abs(left_a) * 1000))
        r = bytes((COMM_SET_CURRENT_BRAKE,)) + struct.pack(">i", round(abs(right_a) * 1000))
        with self.io_lock:
            self.send(l); self.send(self.fwd(r))

    def handbrake(self, left_a: float, right_a: float):
        l = bytes((COMM_SET_HANDBRAKE,)) + struct.pack(">i", round(abs(left_a) * 1000))
        r = bytes((COMM_SET_HANDBRAKE,)) + struct.pack(">i", round(abs(right_a) * 1000))
        with self.io_lock:
            self.send(l); self.send(self.fwd(r))

    def values(self, right=False) -> Values:
        req = bytes((COMM_GET_VALUES_SELECTIVE,)) + struct.pack(">I", VALUE_MASK)
        expected_id = RIGHT_ID if right else 1
        deadline = time.monotonic() + self.timeout
        with self.io_lock:
            self.send(self.fwd(req) if right else req)
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"no selective values reply from VESC ID {expected_id}")
                p = self.recv(COMM_GET_VALUES_SELECTIVE, remaining)
                v = parse_selective(p)
                if v.vesc_id == expected_id:
                    return v

    def detect_hall(self, current_a: float = 1.0, right: bool = False):
        if current_a <= 0.0:
            raise ValueError("Hall detect current must be > 0 A")
        req = bytes((COMM_DETECT_HALL_FOC,)) + struct.pack(">i", round(current_a * 1000.0))
        p = self.transact(self.fwd(req) if right else req, COMM_DETECT_HALL_FOC, 20.0)
        if len(p) != 10:
            raise ValueError(f"unexpected Hall detect reply length {len(p)}")
        table = list(p[1:9])
        ok = p[9] == 0
        return ok, table

    @staticmethod
    def _custom(op: int, data: bytes = b"") -> bytes:
        return bytes((COMM_CUSTOM_APP_DATA,)) + HB_MAGIC + bytes((HB_VERSION, op)) + data

    def custom_transact(self, op: int, data: bytes = b"", right: bool = False,
                        timeout: float | None = None) -> bytes:
        req = self._custom(op, data)
        return self.transact(self.fwd(req) if right else req, COMM_CUSTOM_APP_DATA, timeout)

    def position_state(self, right: bool = False) -> PositionState:
        return parse_position_state(self.custom_transact(HB_GET_POS_STATE, right=right), HB_GET_POS_STATE)

    def set_position_limits(self, minimum: int, maximum: int, right: bool = False) -> PositionState:
        if not (-2147483648 <= minimum <= 2147483647 and -2147483648 <= maximum <= 2147483647):
            raise ValueError("position limits must fit signed int32")
        if minimum > maximum:
            raise ValueError("minimum must be <= maximum")
        p = self.custom_transact(HB_SET_POS_LIMITS, struct.pack(">ii", minimum, maximum), right=right)
        return parse_position_state(p, HB_SET_POS_LIMITS)

    def set_position_counts(self, target: int, right: bool = False) -> PositionState:
        if not -2147483648 <= target <= 2147483647:
            raise ValueError("position target must fit signed int32")
        p = self.custom_transact(HB_SET_POS_TARGET, struct.pack(">i", target), right=right)
        return parse_position_state(p, HB_SET_POS_TARGET)

    def reset_position(self, right: bool = False) -> PositionState:
        return parse_position_state(self.custom_transact(HB_RESET_POSITION, right=right), HB_RESET_POSITION)

    def diag(self, right: bool = False) -> Diag:
        # Diagnostic reply is now ~122 bytes. At 115200 baud it can overlap a
        # 40-60 Hz command refresher, so allow one longer transaction/retry.
        last = None
        for _ in range(2):
            try:
                return parse_diag(self.custom_transact(HB_GET_DIAG, right=right, timeout=max(self.timeout, 0.60)))
            except TimeoutError as exc:
                last = exc
        raise last


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
        try:
            dl, dr = self.link.diag(False), self.link.diag(True)
            self.stop_erpm_l = 5 * (dl.pole_pairs or POLE_PAIRS)
            self.stop_erpm_r = 5 * (dr.pole_pairs or POLE_PAIRS)
        except Exception:
            self.stop_erpm_l = self.stop_erpm_r = STOP_ERPM
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def set(self, mode: str, left: float, right: float):
        with self.lock:
            self.mode, self.left, self.right = mode, left, right
            self.active, self.stop_flag = True, False

    def stop_controlled(self):
        with self.lock:
            # Speed uses firmware RPM ramp-to-zero. Current/duty/position issue
            # zero current once and stop refreshing; the 500-ms ownership timeout
            # then releases the bridge to free-run.
            if self.mode == "rpm":
                self.left = 0.0
                self.right = 0.0
                self.stop_flag = True
                self.active = True
            else:
                self.stop_flag = False
                self.active = False
        if self.mode != "rpm":
            self.link.set_current(0.0, 0.0)

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
                    if stopping and mode == "rpm":
                        self.link.set_rpm(0, 0)
                    elif mode == "current": self.link.set_current(l, r)
                    elif mode == "rpm": self.link.set_rpm(int(l), int(r))
                    elif mode == "duty": self.link.set_duty(l, r)
                    elif mode == "pos": self.link.set_pos(l, r)
                if now >= next_tel:
                    next_tel = now + self.telemetry_period
                    self.last_l = self.link.values(False)
                    self.last_r = self.link.values(True)
                    if stopping and mode == "rpm" and abs(self.last_l.rpm) <= self.stop_erpm_l and abs(self.last_r.rpm) <= self.stop_erpm_r:
                        self.link.set_rpm(0, 0)
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
    ap.add_argument("--telemetry-hz", type=float, default=50.0,
                    help="selective VESC telemetry polling; default 50 Hz")
    args = ap.parse_args()
    link = VescDual(args.port, args.baud)
    try:
        print("local:", parse_fw(link.fw(False)))
        print("virtual CAN:", link.ping_can())
        print("right:", parse_fw(link.fw(True)))
        w = ReplWorker(link, args.command_hz, args.telemetry_hz)
        print("commands: current L R [A] | rpm L R [ERPM] | duty L R [-1..1] | pos L R [deg] | hall [A] [left|right|both] | diag | posstate | stop | release | values | scan | fw | quit")
        while True:
            try: line = input("vesc-dual> ").strip()
            except (EOFError, KeyboardInterrupt): break
            if not line: continue
            a = line.split(); cmd = a[0].lower()
            try:
                if cmd == "current" and len(a) == 3: w.set("current", float(a[1]), float(a[2]))
                elif cmd == "rpm" and len(a) == 3: w.set("rpm", float(a[1]), float(a[2]))
                elif cmd == "duty" and len(a) == 3: w.set("duty", float(a[1]), float(a[2]))
                elif cmd in ("pos", "position") and len(a) == 3: w.set("pos", float(a[1]), float(a[2]))
                elif cmd == "stop": w.stop_controlled()
                elif cmd == "release": w.release()
                elif cmd == "values": print("L", w.last_l.short()); print("R", w.last_r.short())
                elif cmd == "scan": print("virtual CAN IDs:", link.ping_can())
                elif cmd == "fw": print("L", parse_fw(link.fw(False))); print("R", parse_fw(link.fw(True)))
                elif cmd == "diag": print("L", link.diag(False).short()); print("R", link.diag(True).short())
                elif cmd == "posstate": print("L", link.position_state(False)); print("R", link.position_state(True))
                elif cmd == "hall":
                    current = float(a[1]) if len(a) >= 2 else 1.0
                    which = a[2].lower() if len(a) >= 3 else "both"
                    if which in ("left", "l", "both"):
                        ok, tab = link.detect_hall(current, False); print("Hall LEFT", "OK" if ok else "FAIL", tab)
                    if which in ("right", "r", "both"):
                        ok, tab = link.detect_hall(current, True); print("Hall RIGHT", "OK" if ok else "FAIL", tab)
                elif cmd in ("quit", "exit", "q"): break
                else: print("usage: current L R | rpm L R | duty L R | pos L R | hall [A] [left|right|both] | stop | release | values | scan | fw | quit")
            except Exception as e:
                print("[ERR]", e)
        w.shutdown()
    finally:
        link.close()

if __name__ == "__main__":
    main()
