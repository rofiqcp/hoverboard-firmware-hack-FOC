#!/usr/bin/env python3
"""Tes motor kanan saja melalui USART3, mode SPD firmware."""
import argparse
import struct
import time
import serial

START = 0xABCD
COMMAND = struct.Struct('<HhhH')
# start, Lrpm, Lpos, Lcurrent, Lerror, Rrpm, Rpos, Rerror, checksum
FEEDBACK = struct.Struct('<HhhhhhhhH')


def checksum(values):
    result = 0
    for value in values:
        result ^= value & 0xFFFF
    return result & 0xFFFF


def send_command(port, speed):
    steer = 0
    port.write(COMMAND.pack(START, steer, speed,
                            checksum((START, steer, speed))))


def read_feedback(port):
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
        send_command(port, 0)
        time.sleep(0.02)


def main():
    parser = argparse.ArgumentParser(description='Tes motor kanan saja')
    parser.add_argument('port', nargs='?', default='/dev/ttyUSB0')
    parser.add_argument('--speed', type=int, default=100)
    parser.add_argument('--duration', type=float, default=5.0)
    args = parser.parse_args()

    if not -1000 <= args.speed <= 1000:
        parser.error('--speed harus -1000..1000')
    if not 0 < args.duration <= 15:
        parser.error('--duration harus >0 dan maksimal 15 detik')

    with serial.Serial(args.port, 115200, timeout=0.02) as port:
        try:
            print(f'Tes RIGHT: command={args.speed}, duration={args.duration:.1f}s')
            started = time.monotonic()
            last_send = 0.0
            last_print = 0.0
            max_rpm = 0
            moved = False

            while time.monotonic() - started < args.duration:
                now = time.monotonic()
                if now - last_send >= 0.05:
                    send_command(port, args.speed)
                    last_send = now

                values = read_feedback(port)
                if values is None:
                    continue
                _, lrpm, lpos, lcurrent, lerr, rrpm, rpos, rerr, _ = values
                max_rpm = max(max_rpm, abs(rrpm))
                moved |= abs(rrpm) >= 5

                if now - last_print >= 0.1:
                    print(f'Rrpm={rrpm:6d} Rpos={rpos:6d} Rerr={rerr:3d} | '
                          f'Lrpm={lrpm:6d} Lpos={lpos:6d} Lcurrent={lcurrent/100:5.2f}A')
                    last_print = now
        finally:
            stop(port)

    print(f'STOP dikirim. Max Rrpm={max_rpm}. Motor kanan {"BERGERAK" if moved else "TIDAK BERGERAK"}.')


if __name__ == '__main__':
    main()
