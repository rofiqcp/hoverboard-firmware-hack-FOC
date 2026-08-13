#!/usr/bin/env python3
import argparse, struct, time
import serial

START = 0xABCD
FRAME = struct.Struct('<HhhhhhhH')
# start, Lrpm, Lpos, Lerrorcode, Rrpm, Rpos, Rerrorcode, checksum

def xor_checksum(vals):
    c = 0
    for v in vals:
        c ^= v & 0xFFFF
    return c & 0xFFFF

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

def main():
    ap = argparse.ArgumentParser(description='Read hoverboard USART feedback RPM')
    ap.add_argument('port', nargs='?', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    args = ap.parse_args()
    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        while True:
            vals = read_frame(ser)
            if not vals:
                continue
            _, rpm_l, pos_l, err_l, rpm_r, pos_r, err_r, _ = vals
            print(f'Lrpm {rpm_l:6d}  Lpos {pos_l:6d}  Lerrorcode {err_l:3d}  Rrpm {rpm_r:6d}  Rpos {pos_r:6d}  Rerrorcode {err_r:3d}', flush=True)

if __name__ == '__main__':
    main()
