#!/usr/bin/env python3
import argparse, socket, struct, sys, time
from pathlib import Path

COMM_FW_VERSION=0; COMM_JUMP_TO_BOOTLOADER=1; COMM_ERASE_NEW_APP=2; COMM_WRITE_NEW_APP_DATA=3
MAX_FW=120*1024-6

def crc16(data: bytes)->int:
    crc=0
    for x in data:
        crc ^= x<<8
        for _ in range(8): crc=((crc<<1)^0x1021)&0xffff if crc&0x8000 else (crc<<1)&0xffff
    return crc

def frame(payload: bytes)->bytes:
    n=len(payload)
    h=bytes((2,n)) if n<=255 else bytes((3,(n>>8)&255,n&255))
    c=crc16(payload)
    return h+payload+bytes((c>>8,c&255,3))

class Link:
    def __init__(self,args):
        self.args=args; self.sock=None; self.ser=None; self.buf=bytearray()
        self.open()
    def open(self):
        if self.args.transport=='tcp':
            self.sock=socket.create_connection((self.args.host,self.args.port),timeout=3); self.sock.settimeout(.1)
        else:
            import serial
            self.ser=serial.Serial(self.args.serial_port,self.args.baud,timeout=.1,write_timeout=2)
    def close(self):
        try:
            if self.sock: self.sock.close()
            if self.ser: self.ser.close()
        except Exception: pass
    def write(self,b):
        if self.sock: self.sock.sendall(b)
        else: self.ser.write(b); self.ser.flush()
    def read_some(self):
        try:
            d=self.sock.recv(4096) if self.sock else self.ser.read(4096)
            if d: self.buf.extend(d)
        except (socket.timeout,TimeoutError): pass
    def recv_payload(self,timeout=2.0):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.read_some()
            while self.buf and self.buf[0] not in (2,3): del self.buf[0]
            if len(self.buf)<2: continue
            if self.buf[0]==2:
                n=self.buf[1]; hdr=2
            else:
                if len(self.buf)<3: continue
                n=(self.buf[1]<<8)|self.buf[2]; hdr=3
            total=hdr+n+3
            if len(self.buf)<total: continue
            raw=bytes(self.buf[:total]); del self.buf[:total]
            p=raw[hdr:hdr+n]; got=(raw[hdr+n]<<8)|raw[hdr+n+1]
            if raw[-1]==3 and got==crc16(p): return p
        raise TimeoutError('VESC response timeout')
    def transact(self,payload,expected,timeout=3.0):
        self.write(frame(payload)); end=time.monotonic()+timeout
        while time.monotonic()<end:
            p=self.recv_payload(max(.05,end-time.monotonic()))
            if p and p[0]==expected: return p
        raise TimeoutError(f'no response id={expected}')

def fw_version(link,timeout=2.0):
    p=link.transact(bytes((COMM_FW_VERSION,)),COMM_FW_VERSION,timeout)
    if len(p)<4: return 'unknown'
    z=p.find(b'\0',3); return p[3:z if z>=0 else len(p)].decode(errors='replace')

def upload(link,fw:bytes):
    if not fw or len(fw)>MAX_FW: raise RuntimeError(f'firmware size {len(fw)} exceeds {MAX_FW}')
    hw=fw_version(link,4); print(f'[VESC] connected: {hw}')
    p=link.transact(bytes((COMM_ERASE_NEW_APP,))+struct.pack('>I',len(fw)),COMM_ERASE_NEW_APP,12)
    if len(p)<2 or p[1]!=1: raise RuntimeError('erase staging rejected')
    staged=struct.pack('>IH',len(fw),crc16(fw))+fw
    for off in range(0,len(staged),384):
        chunk=staged[off:off+384]
        p=link.transact(bytes((COMM_WRITE_NEW_APP_DATA,))+struct.pack('>I',off)+chunk,COMM_WRITE_NEW_APP_DATA,4)
        if len(p)<6 or p[1]!=1 or struct.unpack('>I',p[2:6])[0]!=off: raise RuntimeError(f'write failed at {off}')
        if off==0 or off+len(chunk)>=len(staged) or off%(384*20)==0:
            print(f'[VESC] write {min(off+len(chunk),len(staged))}/{len(staged)}')
    link.write(frame(bytes((COMM_JUMP_TO_BOOTLOADER,))))
    print('[VESC] staged CRC OK; bootloader copy requested')
    deadline=time.monotonic()+20
    last=''
    while time.monotonic()<deadline:
        time.sleep(.35)
        try:
            last=fw_version(link,1.0)
            if last and 'bootloader' not in last.lower():
                print(f'[VESC] application returned: {last}'); return
        except Exception: pass
    raise RuntimeError(f'application did not return after update; last={last!r}')

def selftest():
    p=b'\x00\x06\x00test\x00'; f=frame(p)
    assert f[0]==2 and f[1]==len(p) and f[-1]==3
    assert crc16(b'123456789')==0x31C3
    print('PIO_VESC_UPLOADER_SELFTEST_PASS')

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--transport',choices=['serial','tcp'])
    ap.add_argument('--serial-port'); ap.add_argument('--baud',type=int,default=2000000)
    ap.add_argument('--host',default='127.0.0.1'); ap.add_argument('--port',type=int,default=65102)
    ap.add_argument('--firmware'); ap.add_argument('--selftest',action='store_true')
    a=ap.parse_args()
    if a.selftest: selftest(); return
    if not a.transport or not a.firmware: ap.error('--transport and --firmware required')
    if a.transport=='serial' and not a.serial_port: ap.error('--serial-port required')
    fw=Path(a.firmware).read_bytes(); link=Link(a)
    try: upload(link,fw)
    finally: link.close()
if __name__=='__main__':
    try: main()
    except Exception as e:
        print(f'UPLOAD_FAIL: {e}',file=sys.stderr); raise SystemExit(2)
