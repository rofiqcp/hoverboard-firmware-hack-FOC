#!/usr/bin/env python3
import socket, struct, subprocess, sys, tempfile, threading, time
from pathlib import Path

R=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
SCRIPT=R/'tools/pio_vesc_upload.py'

def crc16(data):
    crc=0
    for x in data:
        crc ^= x << 8
        for _ in range(8):
            crc=((crc << 1) ^ 0x1021) & 0xffff if crc & 0x8000 else (crc << 1) & 0xffff
    return crc

def frame(p):
    h=bytes((2,len(p))) if len(p)<=255 else bytes((3,(len(p)>>8)&255,len(p)&255))
    c=crc16(p)
    return h+p+bytes((c>>8,c&255,3))

def recv_frame(c,buf):
    c.settimeout(2.0)
    while True:
        while buf and buf[0] not in (2,3): del buf[0]
        if len(buf)>=2:
            if buf[0]==2:
                n=buf[1]; h=2
            elif len(buf)>=3:
                n=(buf[1]<<8)|buf[2]; h=3
            else:
                n=-1; h=0
            if n>=0 and len(buf)>=h+n+3:
                raw=bytes(buf[:h+n+3]); del buf[:h+n+3]
                p=raw[h:h+n]
                assert raw[-1]==3 and ((raw[h+n]<<8)|raw[h+n+1])==crc16(p)
                return p
        d=c.recv(4096)
        if not d: raise EOFError
        buf.extend(d)

fw=bytes((i*73+19)&255 for i in range(4097))
state={'stage':bytearray(), 'boot':False, 'updated':False, 'app_probes':0,
       'writes':0, 'connections':0, 'dropped':False, 'error':None, 'done':False}
with tempfile.TemporaryDirectory() as td:
    f=Path(td)/'fw.bin'; f.write_bytes(fw)
    srv=socket.socket(); srv.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    srv.bind(('127.0.0.1',0)); srv.listen(4); srv.settimeout(8.0); port=srv.getsockname()[1]

    def worker():
        try:
            while not state['done']:
                try:
                    c,_=srv.accept()
                except socket.timeout:
                    raise TimeoutError('server timed out waiting for reconnect')
                state['connections'] += 1
                buf=bytearray(); drop_this_connection=False
                try:
                    while not state['done']:
                        p=recv_frame(c,buf); cmd=p[0]
                        if cmd==0:
                            if state['updated']: name=b'motor_left_updated\0'
                            elif state['boot']: name=b'f103rc_bootloader\0'
                            else: name=b'motor_left\0'
                            c.sendall(frame(bytes((0,6,0))+name))
                            if state['updated']:
                                state['app_probes'] += 1
                                if state['app_probes'] >= 2: state['done']=True
                        elif cmd==1:
                            if not state['boot']:
                                state['boot']=True
                            else:
                                size,crc=struct.unpack('>IH',state['stage'][:6])
                                assert size==len(fw) and crc==crc16(fw)
                                assert bytes(state['stage'][6:6+size])==fw
                                state['updated']=True; state['boot']=False
                        elif cmd==2:
                            assert state['boot']
                            sz=struct.unpack('>I',p[1:5])[0]; assert sz==len(fw)
                            state['stage']=bytearray(b'\xff'*(len(fw)+6))
                            c.sendall(frame(bytes((2,1))))
                        elif cmd==3:
                            assert state['boot']
                            off=struct.unpack('>I',p[1:5])[0]; data=p[5:]
                            state['stage'][off:off+len(data)]=data; state['writes']+=1
                            # Fault injection: commit one idempotent block, then lose its ACK
                            # and tear down the TCP route. The uploader must reconnect, probe
                            # the bootloader, retry the same offset, and finish safely.
                            if not state['dropped'] and off >= 384:
                                state['dropped']=True; drop_this_connection=True
                                try: c.shutdown(socket.SHUT_RDWR)
                                except OSError: pass
                                c.close(); break
                            c.sendall(frame(bytes((3,1))+struct.pack('>I',off)))
                        else:
                            raise AssertionError(cmd)
                except (EOFError, ConnectionError, BrokenPipeError, socket.timeout, OSError):
                    pass
                finally:
                    if not drop_this_connection:
                        try: c.close()
                        except OSError: pass
        except Exception as e:
            state['error']=repr(e)

    th=threading.Thread(target=worker,daemon=True); th.start()
    r=subprocess.run([sys.executable,str(SCRIPT),'--transport','tcp','--host','127.0.0.1',
                      '--port',str(port),'--firmware',str(f)],capture_output=True,text=True,timeout=25)
    th.join(3); srv.close()
    if r.returncode:
        print(r.stdout+r.stderr); raise SystemExit(r.returncode)
    assert state['error'] is None,state['error']
    assert state['dropped'] and state['connections'] >= 2,state
    assert state['updated'] and state['app_probes'] >= 2,state
    assert 'transport recovery: offset=' in r.stdout,r.stdout
    assert 'recovery probe bootloader ready: f103rc_bootloader' in r.stdout,r.stdout
    assert 'application returned stable: motor_left_updated' in r.stdout,r.stdout
print(f'PIO_VESC_UPLOADER_RECOVERY_PASS connections={state["connections"]} writes={state["writes"]} dropped={state["dropped"]}')
