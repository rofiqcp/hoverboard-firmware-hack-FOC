#!/usr/bin/env python3
"""
Tes motor kanan dengan auto-recovery watch.
"""
import argparse
import struct
import time
import serial

START = 0xABCD
COMMAND = struct.Struct('<HhhH')
FEEDBACK = struct.Struct('<HhhhhhhhH')


def checksum(values):
    result = 0
    for value in values:
        result ^= value & 0xFFFF
    return result & 0xFFFF


def send(port, speed):
    steer = 0
    port.write(COMMAND.pack(START, steer, speed,
                            checksum((START, steer, speed))))


def read(port):
    marker = struct.pack('<H', START)
    sync = bytearray()
    while True:
        byte = port.read(1)
        if not byte:
            return None
        sync += byte
        if sync[-2:] == marker:
            tail = port.read(FEEDBACK.size - 2)
            if len(tail) != FEEDBACK.size - 2:
                return None
            values = FEEDBACK.unpack(marker + tail)
            if checksum(values[:-1]) == values[-1]:
                return values


def stop(port):
    for _ in range(5):
        send(port, 0)
        time.sleep(0.02)


def main():
    ap = argparse.ArgumentParser(description='Tes motor kanan dengan recovery watch')
    ap.add_argument('port', nargs='?', default='/dev/ttyUSB0')
    ap.add_argument('--speed', type=int, default=100)
    ap.add_argument('--timeout', type=float, default=10.0)
    args = ap.parse_args()

    with serial.Serial(args.port, 115200, timeout=0.02) as port:
        print(f'[START] speed={args.speed}, timeout={args.timeout}s')
        t0 = time.monotonic()
        last_send = 0.0
        max_rpm = 0
        recovery_count = 0
        prev_err = 0

        try:
            while time.monotonic() - t0 < args.timeout:
                now = time.monotonic()
                if now - last_send >= 0.05:
                    send(port, args.speed)
                    last_send = now

                vals = read(port)
                if vals is None:
                    continue

                _, lrpm, lpos, lcurr, lerr, rrpm, rpos, rerr, _ = vals
                max_rpm = max(max_rpm, abs(rrpm))

                # Detect recovery from error 4 → 0
                if prev_err == 4 and rerr == 0:
                    recovery_count += 1
                    print(f'  [RECOVERY #{recovery_count}] rerr=0 -> motor aktif lagi')

                prev_err = rerr

                # Print lines with status
                if rerr == 4:
                    print(f'  rerr=4 ERROR, Rrpm={rrpm:5d} Rpos={rpos:6d} | recovery dalam ~3 detik')
                elif recovery_count > 0:
                    print(f'  rerr={rerr:2d} Rrpm={rrpm:5d} Rpos={rpos:6d} [recovery #{recovery_count}]')
                elif rrpm != 0:
                    print(f'  rerr={rerr:2d} Rrpm={rrpm:5d} Rpos={rpos:6d}')
        finally:
            stop(port)

    print(f'\n[SELESAI] Max Rrpm={max_rpm}')
    print(f'Recovery count: {recovery_count}')
    if max_rpm > 5:
        print('MOTOR BERGERAK - DONE')


if __name__ == '__main__':
    main()
