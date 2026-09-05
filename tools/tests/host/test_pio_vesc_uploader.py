#!/usr/bin/env python3
import socket, struct, subprocess, sys, tempfile, threading, time
from pathlib import Path
R=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
SCRIPT=R/'tools/pio_vesc_upload.py'

def crc16(data):
    crc=0
    for x in data:
        crc ^= x<<8
        for _ in range(8): crc=((crc<<1)^0x1021)&0xffff if crc&0x8000 else (crc<<1)&0xffff
    return crc

def frame(p):
    h=bytes((2,len(p))) if len(p)<=255 else bytes((3,(len(p)>>8)&255,len(p)&255))
    c=crc16(p); return h+p+bytes((c>>8,c&255,3))

def recv_frame(c,buf):
    end=time.time()+3
    while time.time()<end:
        while buf and buf[0] not in (2,3): del buf[0]
        if len(buf)>=2:
            if buf[0]==2: n=buf[1]; h=2
            elif len(buf)>=3: n=(buf[1]<<8)|buf[2]; h=3
            else: n=-1; h=0
            if n>=0 and len(buf)>=h+n+3:
                raw=bytes(buf[:h+n+3]); del buf[:h+n+3]
                p=raw[h:h+n]
                assert raw[-1]==3 and ((raw[h+n]<<8)|raw[h+n+1])==crc16(p)
                return p
        d=c.recv(4096)
        if not d: raise EOFError
        buf.extend(d)
    raise TimeoutError

fw=bytes((i*37+11)&255 for i in range(2049))
state={'stage':bytearray(), 'jump':False, 'writes':0, 'error':None}
with tempfile.TemporaryDirectory() as td:
    f=Path(td)/'fw.bin'; f.write_bytes(fw)
    srv=socket.socket(); srv.bind(('127.0.0.1',0)); srv.listen(1); port=srv.getsockname()[1]
    def worker():
        try:
            c,_=srv.accept(); buf=bytearray()
            while True:
                p=recv_frame(c,buf); cmd=p[0]
                if cmd==0:
                    name=b'motor_left\0' if not state['jump'] else b'motor_left_updated\0'
                    # Minimal payload accepted by uploader: id, major, minor, hw string
                    c.sendall(frame(bytes((0,6,0))+name))
                    if state['jump']: break
                elif cmd==2:
                    sz=struct.unpack('>I',p[1:5])[0]; assert sz==len(fw)
                    state['stage']=bytearray(b'\xff'*(len(fw)+6)); c.sendall(frame(bytes((2,1))))
                elif cmd==3:
                    off=struct.unpack('>I',p[1:5])[0]; data=p[5:]; state['stage'][off:off+len(data)]=data; state['writes']+=1
                    c.sendall(frame(bytes((3,1))+struct.pack('>I',off)))
                elif cmd==1:
                    state['jump']=True
                    size,crc=struct.unpack('>IH',state['stage'][:6]); assert size==len(fw); assert crc==crc16(fw); assert bytes(state['stage'][6:6+size])==fw
                else: raise AssertionError(cmd)
            c.close()
        except Exception as e: state['error']=repr(e)
    th=threading.Thread(target=worker,daemon=True); th.start()
    r=subprocess.run([sys.executable,str(SCRIPT),'--transport','tcp','--host','127.0.0.1','--port',str(port),'--firmware',str(f)],capture_output=True,text=True,timeout=15)
    th.join(2); srv.close()
    if r.returncode: print(r.stdout+r.stderr); raise SystemExit(r.returncode)
    assert state['error'] is None,state['error']; assert state['jump']; assert state['writes']>=6
    assert 'application returned: motor_left_updated' in r.stdout
print(f'PIO_VESC_UPLOADER_MOCK_PASS bytes={len(fw)} writes={state["writes"]} crc=0x{crc16(fw):04x}')
