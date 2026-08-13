#!/usr/bin/env python3
"""
Monitor homing state motor kiri + arus + RPM encoder.
Feedback frame: start(2), Lrpm(2), Lpos(2), currL(2), errL/state(2), Rrpm(2), Rpos(2), Rerr(2), chk(2) = 18 bytes.
State enum: 0=DISABLED, 1=HOMING_START, 2=MOVE_LEFT, 3=LEFT_FOUND, 4=MOVE_RIGHT, 5=RIGHT_FOUND, 6=COMPLETE, 7=FAILED
"""
import struct, sys, time
import serial

FRAME = struct.Struct('<HhhhhhhhH')  # start + 8 fields + checksum = 18 bytes

STATE_NAMES = {
    0: "DISABLED",
    1: "START",
    2: "MOVE_LEFT",
    3: "LEFT_FOUND",
    4: "MOVE_RIGHT",
    5: "RIGHT_FOUND",
    6: "COMPLETE",
    7: "FAILED",
}

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
            return vals

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    print(f"MONITOR HOMING — port {port}")
    print("State: 0=DISABLED 1=START 2=MOVE_LEFT 3=LEFT_FOUND 4=MOVE_RIGHT 5=RIGHT_FOUND 6=COMPLETE 7=FAILED")
    print("-" * 70)

    with serial.Serial(port, 115200, timeout=1) as ser:
        prev_state = -1
        while True:
            vals = read_frame(ser)
            if not vals:
                continue
            _, lrpm, lpos, currL, errL_state, rrpm, rpos, rerr, _ = vals
            state = errL_state
            curr_A = currL / 100.0

            state_name = STATE_NAMES.get(state, f"UNK({state})")
            changed = state != prev_state
            if changed:
                print(f"\n*** STATE CHANGE: {prev_state} -> {state} ({state_name}) ***")
                prev_state = state

            if state in (2, 4):  # MOVE_LEFT or MOVE_RIGHT
                print(f"  state={state_name:<12} enc={lpos:5d} lrpm={lrpm:5d} curr={curr_A:5.2f}A")
            elif state == 6:
                print(f"  HOMED! enc={lpos:5d} lrpm={lrpm:5d} curr={curr_A:5.2f}A *** COMPLETE ***")
            elif state == 7:
                print(f"  FAILED! enc={lpos:5d} curr={curr_A:5.2f}A *** FAILED ***")

            if state == 6 or state == 7:
                break

            time.sleep(0.05)

if __name__ == '__main__':
    main()
