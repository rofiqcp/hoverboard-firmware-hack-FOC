#!/usr/bin/env python3
import argparse, struct, subprocess
from pathlib import Path

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--firmware',required=True)
    ap.add_argument('--address',required=True,type=lambda x:int(x,0))
    a=ap.parse_args()
    fw=Path(a.firmware).resolve(); data=fw.read_bytes()
    if len(data)<8: raise SystemExit('STLINK_BIN_FAIL: image too small')
    sp,rv=struct.unpack_from('<II',data,0)
    if not (0x20000000 <= sp <= 0x2000C000 and (sp & 3)==0):
        raise SystemExit(f'STLINK_BIN_FAIL: invalid MSP 0x{sp:08X}')
    if (rv & 1)==0: raise SystemExit(f'STLINK_BIN_FAIL: reset vector not Thumb 0x{rv:08X}')
    pc=rv & ~1
    if not (a.address <= pc < a.address+len(data)+0x4000):
        raise SystemExit(f'STLINK_BIN_FAIL: reset vector 0x{pc:08X} outside image')
    pio=Path.home()/'.platformio/packages/tool-openocd'
    ocd=pio/'bin/openocd'; scripts=pio/'openocd/scripts'
    cmd=[str(ocd),'-s',str(scripts),'-f','interface/stlink.cfg','-c','adapter speed 100','-f','target/stm32f1x.cfg',
         '-c',f'program {fw} 0x{a.address:08X} verify reset; shutdown']
    print(f'[STLINK] binary={fw.name} bytes={len(data)} address=0x{a.address:08X} MSP=0x{sp:08X} RV=0x{rv:08X}')
    subprocess.run(cmd,check=True)
    print('STLINK_BIN_UPLOAD_PASS')
if __name__=='__main__': main()
