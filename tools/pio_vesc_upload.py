#!/usr/bin/env python3
import argparse, os, socket, struct, subprocess, sys, time
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
        self.args=args; self.sock=None; self.ser=None; self.buf=bytearray(); self.linebuf=bytearray(); self.f411_direct=False
        self.open()

    def _holders(self, port):
        real=os.path.realpath(port)
        r=subprocess.run(['fuser',real],stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True)
        out=[]
        for tok in r.stdout.split():
            if not tok.isdigit(): continue
            pid=int(tok)
            if pid==os.getpid(): continue
            try: cmd=Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
            except Exception: cmd=''
            out.append((pid,cmd))
        return out

    def _open_tcp(self):
        last=None
        for attempt in range(1,11):
            try:
                self.sock=socket.create_connection((self.args.host,self.args.port),timeout=2)
                self.sock.settimeout(.1); self.buf.clear(); time.sleep(.35)
                return
            except OSError as e:
                last=e; self.sock=None
                if attempt<10: time.sleep(.25)
        raise RuntimeError(f'cannot connect F411 ROS gateway TCP {self.args.host}:{self.args.port}: {last}')

    def reconnect_tcp(self):
        if self.sock:
            try: self.sock.close()
            except Exception: pass
        self.sock=None; self.buf.clear(); time.sleep(.25); self._open_tcp()

    def _f411_read_line(self, timeout=.1):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            nl=self.linebuf.find(b'\n')
            if nl>=0:
                raw=bytes(self.linebuf[:nl]); del self.linebuf[:nl+1]
                return raw.rstrip(b'\r').decode(errors='replace')
            d=self.ser.read(512)
            if d: self.linebuf.extend(d)
        return None

    def _f411_command(self,text,expect=None,timeout=3.0):
        self.ser.write((text+'\n').encode()); self.ser.flush()
        if expect is None: return ''
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            line=self._f411_read_line(.15)
            if line is None: continue
            if line.startswith('VESC:ERR:'): raise RuntimeError(line)
            if expect in line: return line
        raise TimeoutError(f'F411 command timeout: {text}')

    def _open_f411_direct(self):
        import serial
        self.ser=serial.Serial(self.args.serial_port,115200,timeout=.05,write_timeout=2,exclusive=True)
        self.f411_direct=True; self.ser.reset_input_buffer(); self.ser.reset_output_buffer(); self.linebuf.clear()
        self.ser.write(b'\n'); self.ser.flush(); time.sleep(.03); self.ser.reset_input_buffer()
        line=self._f411_command('VESC:MODE:MAINTENANCE','VESC:MODE:MAINTENANCE',3.0)
        print(f'[F411] direct CDC {line}',flush=True)
        self._f411_command('VESC:STATUS','mode=MAINTENANCE',2.0)
        # F411 changes UART ownership synchronously, but allow its CDC/status
        # output and UART RX flush to settle before the first binary packet.
        time.sleep(.30)

    def open(self):
        if self.args.transport=='tcp':
            self._open_tcp(); return
        if self.args.transport=='f411':
            holders=self._holders(self.args.serial_port)
            official=[(pid,cmd) for pid,cmd in holders if 'stmf4_hmi_bridge' in cmd]
            other=[(pid,cmd) for pid,cmd in holders if 'stmf4_hmi_bridge' not in cmd]
            if other:
                pid,cmd=other[0]; raise RuntimeError(f'F411 CDC busy by non-gateway pid={pid}: {cmd[:160]}')
            if official:
                print(f'[F411] CDC owned by stmf4_hmi_bridge pid={official[0][0]}; using local gateway TCP',flush=True)
                self._open_tcp(); return
            self._open_f411_direct(); return
        import serial
        self.ser=serial.Serial(self.args.serial_port,self.args.baud,timeout=.1,write_timeout=2)

    def close(self):
        try:
            if self.sock: self.sock.close()
            if self.ser:
                if self.f411_direct:
                    try: self._f411_command('VESC:MODE:RUNTIME','VESC:MODE:RUNTIME',1.5)
                    except Exception as e: print(f'[F411] runtime restore warning: {e}',file=sys.stderr,flush=True)
                self.ser.close()
        finally:
            self.sock=None; self.ser=None; self.f411_direct=False

    def write(self,b):
        if self.sock:
            self.sock.sendall(b); return
        if self.f411_direct:
            for off in range(0,len(b),48):
                chunk=b[off:off+48]
                self.ser.write(b'VESC:TX:M:'+chunk.hex().upper().encode()+b'\n'); self.ser.flush(); time.sleep(.002)
            return
        self.ser.write(b); self.ser.flush()

    def read_some(self):
        if self.sock:
            try:
                d=self.sock.recv(4096)
                if d: self.buf.extend(d)
                else: raise ConnectionError('TCP bridge closed connection')
            except (socket.timeout,TimeoutError): pass
            return
        if self.f411_direct:
            for _ in range(32):
                line=self._f411_read_line(.01)
                if line is None: break
                if line.startswith('VESC:ERR:'): raise RuntimeError(line)
                if line.startswith('VESC:RX:'):
                    hx=line[8:].strip()
                    try: self.buf.extend(bytes.fromhex(hx))
                    except ValueError: raise RuntimeError(f'bad F411 VESC hex: {hx[:80]}')
            return
        d=self.ser.read(4096)
        if d: self.buf.extend(d)

    def recv_payload(self,timeout=2.0):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.read_some()
            for start in range(len(self.buf)):
                st=self.buf[start]
                if st not in (2,3): continue
                if st==2:
                    if len(self.buf)-start < 2: continue
                    n=self.buf[start+1]; hdr=2
                else:
                    if len(self.buf)-start < 3: continue
                    n=(self.buf[start+1]<<8)|self.buf[start+2]; hdr=3
                if n<=0 or n>4096: continue
                total=hdr+n+3
                if len(self.buf)-start < total: continue
                raw=bytes(self.buf[start:start+total])
                if raw[-1]!=3: continue
                p=raw[hdr:hdr+n]; got=(raw[hdr+n]<<8)|raw[hdr+n+1]
                if got!=crc16(p): continue
                del self.buf[:start+total]
                return p
            if len(self.buf)>8192: del self.buf[:-4096]
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

def wait_for_bootloader(link, initial_hw: str) -> str:
    if 'bootloader' in initial_hw.lower():
        return initial_hw
    print(f'[VESC] application connected: {initial_hw}; entering resident bootloader', flush=True)
    # The application writes only a dual-word SRAM boot request and resets.
    # No flash write and no motor command is issued; recovery starts fail-safe.
    link.write(frame(bytes((COMM_JUMP_TO_BOOTLOADER,))))
    deadline=time.monotonic()+10.0
    last=''
    while time.monotonic()<deadline:
        time.sleep(.25)
        try:
            last=fw_version(link,1.0)
            if 'bootloader' in last.lower():
                print(f'[VESC] bootloader ready: {last}', flush=True)
                return last
        except Exception:
            pass
    raise RuntimeError(f'bootloader did not appear; last={last!r}')


def upload(link,fw:bytes):
    if not fw or len(fw)>MAX_FW: raise RuntimeError(f'firmware size {len(fw)} exceeds {MAX_FW}')
    hw=None; last_error=None
    for attempt in range(1,7):
        try:
            hw=fw_version(link,4); break
        except Exception as e:
            last_error=e
            if link.sock is not None or link.args.transport=='tcp':
                try:
                    link.reconnect_tcp()
                    print(f'[VESC] reconnect initial probe attempt={attempt+1}',flush=True)
                except Exception as re:
                    last_error=re
            else:
                print(f'[VESC] retry initial probe attempt={attempt+1}',flush=True)
                time.sleep(.20)
            continue
    if hw is None: raise RuntimeError(f'initial firmware probe failed: {last_error}')
    wait_for_bootloader(link,hw)
    p=link.transact(bytes((COMM_ERASE_NEW_APP,))+struct.pack('>I',len(fw)),COMM_ERASE_NEW_APP,8)
    if len(p)<2 or p[1]!=1: raise RuntimeError('erase staging rejected')
    staged=struct.pack('>IH',len(fw),crc16(fw))+fw
    step=192  # request payload stays <=255 bytes -> simple short VESC frame
    for off in range(0,len(staged),step):
        chunk=staged[off:off+step]
        request=bytes((COMM_WRITE_NEW_APP_DATA,))+struct.pack('>I',off)+chunk
        last_error=None
        for attempt in range(1,6):
            try:
                p=link.transact(request,COMM_WRITE_NEW_APP_DATA,5)
                if len(p)>=6 and p[1]==1 and struct.unpack('>I',p[2:6])[0]==off:
                    last_error=None; break
                last_error=RuntimeError(f'bad write ACK at {off}: {p.hex()}')
            except Exception as e:
                last_error=e
            if attempt<5:
                print(f'[VESC] retry offset={off} attempt={attempt+1}', flush=True)
                time.sleep(.10)
        if last_error is not None: raise RuntimeError(f'write failed at {off}: {last_error}')
        if off==0 or off+len(chunk)>=len(staged) or off%(step*40)==0:
            print(f'[VESC] write {min(off+len(chunk),len(staged))}/{len(staged)}', flush=True)
        time.sleep(.002)
    link.write(frame(bytes((COMM_JUMP_TO_BOOTLOADER,))))
    print('[VESC] staged CRC complete; bootloader copy requested', flush=True)
    deadline=time.monotonic()+45
    last=''
    while time.monotonic()<deadline:
        time.sleep(.4)
        try:
            last=fw_version(link,1.2)
            if last and 'bootloader' not in last.lower():
                print(f'[VESC] application returned: {last}', flush=True); return
        except Exception:
            pass
    raise RuntimeError(f'application did not return after update; last={last!r}')

def selftest():
    p=b'\x00\x06\x00test\x00'; f=frame(p)
    assert f[0]==2 and f[1]==len(p) and f[-1]==3
    assert crc16(b'123456789')==0x31C3
    print('PIO_VESC_UPLOADER_SELFTEST_PASS')

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--transport',choices=['serial','tcp','f411'])
    ap.add_argument('--serial-port'); ap.add_argument('--baud',type=int,default=1000000)
    ap.add_argument('--host',default='127.0.0.1'); ap.add_argument('--port',type=int,default=65102)
    ap.add_argument('--firmware'); ap.add_argument('--selftest',action='store_true')
    a=ap.parse_args()
    if a.selftest: selftest(); return
    if not a.transport or not a.firmware: ap.error('--transport and --firmware required')
    if a.transport in ('serial','f411') and not a.serial_port: ap.error('--serial-port required')
    fw=Path(a.firmware).read_bytes(); link=Link(a)
    try: upload(link,fw)
    finally: link.close()
if __name__=='__main__':
    try: main()
    except Exception as e:
        print(f'UPLOAD_FAIL: {e}',file=sys.stderr); raise SystemExit(2)
