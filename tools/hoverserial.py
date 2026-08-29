#!/usr/bin/env python3
"""USART3 hoverboard test: drive left/right/both, report RPM and real FOC ISR rate.

Replaces tools/hoverserial.ino (Arduino sketch converted to Python).

Firmware (VARIANT_USART) is configured for USART3 only:
  CONTROL_SERIAL_USART3  -> host sends start/steer/speed frames here
  FEEDBACK_SERIAL_USART3 -> board streams telemetry here, including a 32-bit
                            counter (foc_isr_count) incremented every FOC ISR
                            (DMA1_Channel1_IRQHandler, ~PWM_FREQ = 16000 Hz).

Feedback frame (13 x uint16 = 26 bytes), little-endian, XOR-16 checksum over
the first 12 words:
  start, cmd1, cmd2, rpmR, rpmL, odomR, odomL, battery, temp, led,
  isr_lo, isr_hi, checksum
ISR rate = (isr_now - isr_prev) / dt   [interrupts per second]
"""
import argparse
import struct
import sys
import time

import serial

START = 0xABCD
CMD = struct.Struct("<HhhH")        # start, steer, speed, xor
FB = struct.Struct("<H" + "h" * 8 + "H" * 6)  # 15 words: start(H) cmd1..temp(8xh) led,isrLo,isrHi,cycLast,cycMax,chk(6xH)


def make_command(steer, speed):
    s = int(steer) & 0xFFFF
    v = int(speed) & 0xFFFF
    chk = (START ^ s ^ v) & 0xFFFF
    return CMD.pack(START, int(steer), int(speed), chk)


def signed16(x):
    return x - 65536 if x >= 32768 else x


def decode(buf):
    """Return (remaining_bytes, item_or_None) parsing one valid frame."""
    while len(buf) >= FB.size:
        i = buf.find(struct.pack("<H", START))
        if i < 0:
            return b"", None
        if len(buf) - i < FB.size:
            return buf[i:], None
        raw = buf[i:i + FB.size]
        vals = list(FB.unpack(raw))
        chk = 0
        for w in vals[:-1]:
            chk ^= w & 0xFFFF
        if chk == (vals[-1] & 0xFFFF):
            item = {
                "cmd1": signed16(vals[1]),
                "cmd2": signed16(vals[2]),
                "rpm_r": signed16(vals[3]),
                "rpm_l": signed16(vals[4]),
                "odom_r": signed16(vals[5]),
                "odom_l": signed16(vals[6]),
                "battery": signed16(vals[7]),
                "temp": signed16(vals[8]),
                "led": vals[9],
                "isr": (vals[10] | (vals[11] << 16)) & 0xFFFFFFFF,
                "cyc_last": vals[12],
                "cyc_max": vals[13],
            }
            return buf[i + FB.size:], item
        buf = buf[i + 2:]
    return buf, None


def run_phase(ser, name, steer, speed, seconds, samples, last_isr):
    print(f"\n[{name}] steer={steer} speed={speed} duration={seconds}s")
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        ser.write(make_command(steer, speed))
        deadline = time.monotonic() + 0.06
        got = False
        while time.monotonic() < deadline:
            data = ser.read(FB.size * 2)
            if not data:
                continue
            samples[0] += data
            samples[0], item = decode(samples[0])
            if item:
                got = True
                now = time.monotonic()
                isr_hz = 0.0
                if last_isr[0] is not None:
                    dt = now - last_isr[0]
                    if dt > 0:
                        di = (item["isr"] - last_isr[1]) & 0xFFFFFFFF
                        isr_hz = di / dt
                last_isr[0] = now
                last_isr[1] = item["isr"]
                print(
                    "  RPM L={rpm_l:6d} R={rpm_r:6d} | ISR={isr:10d} ({isr_hz:8.1f}/s) | "
                    "odom L={odom_l:5d} R={odom_r:5d} | cyc last={cyc_last:5d} max={cyc_max:5d}".format(isr_hz=isr_hz, **item)
                )
                break
        if not got:
            time.sleep(0.01)


def measure_isr_rate(ser, seconds, samples):
    """Measure true ISR rate by sampling many frames continuously (no gaps)."""
    pairs = []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        data = ser.read(FB.size * 4)
        if not data:
            continue
        samples[0] += data
        while True:
            samples[0], item = decode(samples[0])
            if item is None:
                break
            pairs.append((time.monotonic(), item["isr"]))
    if len(pairs) < 2:
        return None, 0, len(pairs)
    t0, i0 = pairs[0]
    t1, i1 = pairs[-1]
    dt = t1 - t0
    if dt <= 0:
        return None, 0, len(pairs)
    rate = ((i1 - i0) & 0xFFFFFFFF) / dt
    return rate, len(pairs), len(pairs)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    ap.add_argument("--command", type=int, default=120,
                    help="motor command magnitude [-300..300]")
    ap.add_argument("--ramp", type=float, default=1.0)
    ap.add_argument("--hold", type=float, default=3.0)
    ap.add_argument("--no-motor", action="store_true",
                    help="only observe feedback, do not send drive commands")
    ap.add_argument("--measure", type=float, default=3.0,
                    help="seconds to measure ISR rate before driving")
    args = ap.parse_args()
    if abs(args.command) > 300:
        ap.error("--command must be between -300 and 300")

    print("WARNING: pastikan kedua roda terangkat dan area aman (motor akan berputar).")
    samples = [b""]
    last_isr = [None, 0]

    with serial.Serial(args.port, 115200, timeout=0.02) as ser:
        ser.reset_input_buffer()
        # Accurate ISR-rate measurement while motors are still idle
        rate, packets, _ = measure_isr_rate(ser, args.measure, samples)
        if rate is not None:
            print("\n[ISR MEASURE] rate = {:.1f} interrupts/s over {:.2f}s "
                  "({} packets)".format(rate, args.measure, packets))
        if args.no_motor:
            run_phase(ser, "OBSERVE", 0, 0, 3.0, samples, last_isr)
        else:
            # Safety: hold zero command first so the firmware enables both motors
            # (it only arms when |cmd1| and |cmd2| stay < 50 for one loop pass).
            run_phase(ser, "ENABLE (zero cmd)", 0, 0, 2.0, samples, last_isr)
            # steering mixer: left=(speed+steer), right=(speed-steer)
            run_phase(ser, "LEFT", args.command, args.command, args.ramp, samples, last_isr)
            run_phase(ser, "LEFT HOLD", args.command, args.command, args.hold, samples, last_isr)
            run_phase(ser, "RIGHT", -args.command, args.command, args.ramp, samples, last_isr)
            run_phase(ser, "RIGHT HOLD", -args.command, args.command, args.hold, samples, last_isr)
            run_phase(ser, "BOTH", 0, args.command, args.hold, samples, last_isr)
        # stop burst; keep sending valid frames to avoid serial timeout
        for _ in range(10):
            ser.write(make_command(0, 0))
            time.sleep(0.02)

    print("\nSTOP dikirim. Nilai ISR pada baris di atas = counter kumulatif;")
    print("angka di kurung (x.x/s) = laju ISR aktual (harus ~16000/s).")


if __name__ == "__main__":
    main()
