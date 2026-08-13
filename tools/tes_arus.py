#!/usr/bin/env python3
"""
Tes arus motor left - mode SPD.
Alur:
  1. Set mode SPD
  2. Kirim speed negative ke motor left (minus = arah homing kiri)
  3. Baca arus dari frame feedback
  4. Kalau arus >= threshold, stop motor + catat posisi encoder
  5. Print hasil kalibrasi
"""
import argparse, struct, time
import serial

START = 0xABCD
# Frame baru: start, speedL, posL, currL, errL, speedR, posR, errR, checksum
FRAME = struct.Struct('<HhhhhhhhH')

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

def send_cmd(ser, steer=0, speed=0):
    """Kirim command steer + speed, checksum = start ^ steer ^ speed"""
    frame = struct.pack('<HhhH', START, steer, speed, START ^ (steer & 0xFFFF) ^ (speed & 0xFFFF))
    ser.write(frame)

def set_mode_spd(ser):
    """Mode SPD harus sudah dipilih lewat parameter firmware/GUI."""
    # SerialCommand hanya membawa steer dan speed; mode bukan bagian frame.
    time.sleep(0.05)

def main():
    ap = argparse.ArgumentParser(description='Tes arus motor left - deteksi stopper via arus')
    ap.add_argument('port', nargs='?', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--speed', type=int, default=-200, help='Speed command (negatif = kiri)')
    ap.add_argument('--threshold', type=int, default=150, help='Threshold arus (A*100, default 150=1.5A)')
    ap.add_argument('--timeout', type=float, default=10.0, help='Timeout detik')
    args = ap.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.02) as ser:
        print("[1] Set mode SPD...")
        set_mode_spd(ser)
        time.sleep(0.3)

        # Baca feedback dulu untuk sinkronisasi
        print("[2] Sinkronisasi frame...")
        for _ in range(10):
            read_frame(ser)

        print(f"[3] Kirim speed={args.speed}, watch arus >= {args.threshold}")
        print(f"     Threshold {args.threshold} = {args.threshold/100:.2f} A")
        send_cmd(ser, steer=0, speed=args.speed)

        t_start = time.time()
        stopped = False
        stop_pos = None
        stop_curr = None

        last_cmd = time.monotonic()
        while time.time() - t_start < args.timeout:
            # Refresh command before firmware serial timeout expires.
            if time.monotonic() - last_cmd >= 0.05:
                send_cmd(ser, steer=0, speed=args.speed)
                last_cmd = time.monotonic()
            vals = read_frame(ser)
            if not vals:
                continue

            _, rpm_l, pos_l, curr_l, err_l, rpm_r, pos_r, err_r, _ = vals
            abs_curr = abs(curr_l) if curr_l is not None else 0

            print(f'  rpmL={rpm_l:6d}  posL={pos_l:6d}  currL={curr_l:6d} ({abs_curr/100:.2f}A)  rpmR={rpm_r:6d}  posR={pos_r:6d}', flush=True)

            if abs_curr >= args.threshold:
                print(f'\n*** STOPPER TERDETEKSI! ***')
                print(f'  Arus  = {curr_l} ({abs_curr/100:.2f} A)')
                print(f'  Posisi = {pos_l}')
                print(f'  RPM    = {rpm_l}')
                stop_pos = pos_l
                stop_curr = curr_l
                stopped = True
                # Stop motor
                send_cmd(ser, steer=0, speed=0)
                print('[4] Motor distop.')
                break

            # Kalau rpm sudah 0 tapi arus masih kecil = motor macet tanpa hantaman
            if abs(rpm_l) < 5 and abs_curr < 30:
                print(f'\n[INFO] Motor diam (rpm={rpm_l}, arus={curr_l}), continue watching...')

        if not stopped:
            print(f'\n[TIMEOUT] Tidak ada hantaman stopper dalam {args.timeout}s')
            send_cmd(ser, steer=0, speed=0)

        print('\n========== HASIL KALIBRASI ==========')
        if stop_pos is not None:
            print(f'  Posisi stopper kiri : {stop_pos}')
            print(f'  Arus saat hantam     : {stop_curr} ({abs(stop_curr)/100:.2f} A)')
            # Konversi posisi encoder ke rotations
            # 1024 PPR x4 = 4096 count/rev
            rev = stop_pos / 4096
            print(f'  Rotation              : {rev:.3f} rev')
            print(f'  Half-turn (center)    : {stop_pos // 2}')
        else:
            print('  GAGAL - tidak ada data stopper')
        print('======================================')

        # Biarkan motor mati sebentar sebelum exit
        time.sleep(0.5)

if __name__ == '__main__':
    main()
