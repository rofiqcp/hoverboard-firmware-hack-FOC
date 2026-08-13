#!/usr/bin/env python3
"""
Duty test - kirim pwml=-1000 (duty penuh) lalu watch rpm dan arus.
"""
import argparse, struct, time, serial

START = 0xABCD

def xor_checksum(vals):
    c = 0
    for v in vals:
        c ^= v & 0xFFFF
    return c & 0xFFFF

FRAME = struct.Struct('<HhhhhhhhH')

def read_frame(port):
    buf = bytearray()
    while True:
        b = port.read(1)
        if not b:
            return None
        buf += b
        if len(buf) >= 2 and buf[-2:] == b'\xcd\xab':
            rest = port.read(FRAME.size - 2)
            if len(rest) != FRAME.size - 2:
                return None
            data = bytes(buf[-2:]) + rest
            vals = FRAME.unpack(data)
            if vals[0] == START and xor_checksum(vals[:-1]) == vals[-1]:
                return vals

def send_raw(ser, steer=0, speed=0):
    c = START ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)
    frame = struct.pack('<HhhH', START, steer, speed, c)
    ser.write(frame)

def main():
    ap = argparse.ArgumentParser(description='Duty test motor left')
    ap.add_argument('port', nargs='?', default='/dev/ttyUSB0')
    ap.add_argument('--duty', type=int, default=-1000, help='Duty command [-1000,1000]')
    ap.add_argument('--timeout', type=float, default=5.0)
    args = ap.parse_args()

    with serial.Serial(args.port, 115200, timeout=0.02) as ser:
        print(f"[1] Sync frames...")
        for _ in range(10):
            read_frame(ser)
        print(f"[2] Kirim duty={args.duty} (continuous)...")
        t0 = time.time()
        last_send = 0
        max_curr = 0
        max_rpm = 0

        while time.time() - t0 < args.timeout:
            if time.monotonic() - last_send >= 0.05:
                send_raw(ser, 0, args.duty)
                last_send = time.monotonic()
            vals = read_frame(ser)
            if not vals:
                continue
            _, rpm_l, pos_l, curr_l, err_l, rpm_r, pos_r, err_r, _ = vals
            max_curr = max(max_curr, abs(curr_l))
            max_rpm = max(max_rpm, abs(rpm_l))
            print(f'  rpmL={rpm_l:6d}  posL={pos_l:6d}  currL={curr_l:6d} ({curr_l/100:.2f}A)  maxCurr={max_curr}  maxRpm={max_rpm}', flush=True)

        print(f'\nMax arus : {max_curr} ({max_curr/100:.2f}A)')
        print(f'Max RPM  : {max_rpm}')
        if max_rpm > 10:
            print('MOTOR BERPUTAR')
        elif max_curr > 50:
            print('ARUS ADA TAPI RPM=0 - MOTOR macet/stall')
        else:
            print('MOTOR TIDAK RESPONS - cek enable/pwm/moe')
        send_raw(ser, 0, 0)

if __name__ == '__main__':
    main()
