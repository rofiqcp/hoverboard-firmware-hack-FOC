#!/usr/bin/env python3
import argparse, fcntl, os, signal, socket, struct, subprocess, sys, time
from pathlib import Path

COMM_FW_VERSION=0; COMM_JUMP_TO_BOOTLOADER=1; COMM_ERASE_NEW_APP=2; COMM_WRITE_NEW_APP_DATA=3
MAX_FW=120*1024-6

ROS_PROCESS_MARKERS = (
    "/opt/ros/",
    "/home/otomasi/ros/install/",
    "ros2 launch ",
    "ros2 run ",
)

def _protected_pids():
    protected={os.getpid()}
    pid=os.getppid()
    while pid>1 and pid not in protected:
        protected.add(pid)
        try:
            raw=Path(f'/proc/{pid}/stat').read_text().split()
            pid=int(raw[3])
        except Exception:
            break
    return protected

def _cmdline(pid):
    try:
        return Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace').strip()
    except Exception:
        return ''

def stop_ros_processes():
    protected=_protected_pids(); victims=[]
    for ent in Path('/proc').iterdir():
        if not ent.name.isdigit(): continue
        pid=int(ent.name)
        if pid in protected: continue
        cmd=_cmdline(pid)
        if cmd and any(m in cmd for m in ROS_PROCESS_MARKERS):
            victims.append((pid,cmd))
    if not victims:
        print('[UPLOAD] ROS stack already stopped', flush=True); return
    pids=[pid for pid,_ in victims]
    print(f'[UPLOAD] stopping ROS processes: {pids}', flush=True)
    for pid in pids:
        try: os.kill(pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError): pass
    deadline=time.monotonic()+4.0
    while time.monotonic()<deadline:
        alive=[pid for pid in pids if Path(f'/proc/{pid}').exists()]
        if not alive: return
        time.sleep(.10)
    alive=[pid for pid in pids if Path(f'/proc/{pid}').exists()]
    if alive:
        print(f'[UPLOAD] force-stopping remaining ROS processes: {alive}', flush=True)
    for pid in alive:
        try: os.kill(pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError): pass
    time.sleep(.20)

class RestartUploadSession(RuntimeError):
    pass

class UploadProcessLock:
    def __init__(self, key: str):
        safe=''.join(c if c.isalnum() else '_' for c in key)[:96]
        self.path=Path('/tmp')/f'pio_vesc_upload_{safe}.lock'
        self.fd=None
    def __enter__(self):
        self.fd=os.open(self.path, os.O_CREAT|os.O_RDWR, 0o660)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            try:
                os.lseek(self.fd,0,os.SEEK_SET); owner=os.read(self.fd,128).decode(errors='replace').strip()
            except Exception:
                owner=''
            raise RuntimeError(f'another firmware uploader is already active{f" ({owner})" if owner else ""}')
        os.ftruncate(self.fd,0); os.write(self.fd, f'pid={os.getpid()} started={time.time():.3f}'.encode()); os.fsync(self.fd)
        return self
    def __exit__(self, exc_type, exc, tb):
        if self.fd is not None:
            try: fcntl.flock(self.fd, fcntl.LOCK_UN)
            finally: os.close(self.fd); self.fd=None

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
        self.args=args; self.sock=None; self.ser=None; self.buf=bytearray(); self.linebuf=bytearray(); self.f411_direct=False; self.last_maintenance_refresh=0.0
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

    def _open_tcp(self, attempts=2):
        last=None
        for attempt in range(1, attempts + 1):
            try:
                self.sock=socket.create_connection((self.args.host,self.args.port),timeout=.75)
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.sock.settimeout(.10); self.buf.clear(); time.sleep(.12)
                return
            except OSError as e:
                last=e; self.sock=None
                if attempt < attempts: time.sleep(.12)
        raise RuntimeError(f'cannot connect F411 ROS gateway TCP {self.args.host}:{self.args.port}: {last}')

    def _tcp_probe(self, timeout=.8):
        try:
            p=self.transact(bytes((COMM_FW_VERSION,)),COMM_FW_VERSION,timeout)
            return bool(p and p[0]==COMM_FW_VERSION)
        except Exception:
            return False

    def _stop_official_bridge(self, official):
        if not official: return
        pids=[pid for pid,_ in official]
        print(f'[F411] stopping CDC holder pid={pids} for direct USB upload',flush=True)
        for pid in pids:
            try: os.kill(pid, signal.SIGTERM)
            except ProcessLookupError: pass
        deadline=time.monotonic()+4.0
        while time.monotonic()<deadline:
            remain=[x for x in self._holders(self.args.serial_port) if x[0] in pids]
            if not remain: return
            time.sleep(.10)
        for pid in pids:
            try: os.kill(pid, signal.SIGKILL)
            except ProcessLookupError: pass
        deadline=time.monotonic()+2.0
        while time.monotonic()<deadline:
            if not any(x[0] in pids for x in self._holders(self.args.serial_port)): return
            time.sleep(.10)
        raise RuntimeError(f'F411 CDC still held after stopping pid={pids}')

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

    def _refresh_f411_maintenance(self, force=False):
        if not self.f411_direct or self.ser is None:
            return
        now=time.monotonic()
        if not force and now-self.last_maintenance_refresh < 2.0:
            return
        # Refresh only between complete VESC request/reply transactions. Older
        # F411 gateway builds used a short maintenance lease. Re-entering the
        # same mode is idempotent and keeps direct USB firmware uploads alive.
        line=self._f411_command('VESC:MODE:MAINTENANCE','VESC:MODE:MAINTENANCE',2.0)
        if line.strip() != 'VESC:MODE:MAINTENANCE':
            raise RuntimeError(f'F411 maintenance handshake invalid: {line!r}')
        self.last_maintenance_refresh=time.monotonic()
        if force:
            status=self._f411_command('VESC:STATUS','mode=MAINTENANCE',2.0)
            if 'VESC:STAT:' not in status or 'mode=MAINTENANCE' not in status:
                raise RuntimeError(f'F411 maintenance status invalid: {status!r}')

    def _open_f411_direct(self):
        import serial
        self.ser=serial.Serial(self.args.serial_port,1000000,timeout=.05,write_timeout=2,exclusive=True)
        self.f411_direct=True; self.ser.reset_input_buffer(); self.ser.reset_output_buffer(); self.linebuf.clear()
        self.ser.write(b'\n'); self.ser.flush(); time.sleep(.03); self.ser.reset_input_buffer()
        self._refresh_f411_maintenance(force=True)
        print('[F411] direct CDC maintenance confirmed',flush=True)
        # F411 changes UART ownership synchronously, but allow its CDC/status
        # output and UART RX flush to settle before the first binary packet.
        time.sleep(.20)

    def open(self):
        if self.args.transport=='tcp':
            self._open_tcp(); return
        if self.args.transport=='f411':
            # APP_F411 is intentionally USB-only. Firmware upload must not depend
            # on ROS, TCP port 65101/65102, or vesc_tool_bridge being alive.
            stop_ros_processes()
            holders=self._holders(self.args.serial_port)
            if holders:
                pids=[pid for pid,_ in holders]
                print(f'[F411] releasing CDC holders pid={pids}',flush=True)
                self._stop_official_bridge(holders)
            print(f'[F411] using direct USB CDC only: {self.args.serial_port} @ 1000000',flush=True)
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
            self._refresh_f411_maintenance()
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
    deadline=time.monotonic()+20.0
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


def _recover_bootloader_transport(link, reason: str):
    print(f'[VESC] transport recovery: {reason}', flush=True)
    last_error=None
    for attempt in range(1,5):
        try:
            if link.sock is not None or link.args.transport in ('tcp','f411'):
                if link.sock is not None:
                    link.reconnect_tcp()
                elif link.args.transport=='tcp':
                    link._open_tcp(attempts=4)
            hw=fw_version(link,3.0)
            if 'bootloader' in hw.lower():
                print(f'[VESC] recovery probe bootloader ready: {hw}', flush=True)
                return
            wait_for_bootloader(link,hw)
            raise RestartUploadSession('target application restarted; staging session must restart from erase')
        except RestartUploadSession:
            raise
        except Exception as e:
            last_error=e
            time.sleep(.20*attempt)
    raise RuntimeError(f'transport recovery failed: {last_error}')


def _stage_once(link,fw:bytes,session:int):
    p=link.transact(bytes((COMM_ERASE_NEW_APP,))+struct.pack('>I',len(fw)),COMM_ERASE_NEW_APP,10)
    if len(p)<2 or p[1]!=1: raise RuntimeError('erase staging rejected')
    staged=struct.pack('>IH',len(fw),crc16(fw))+fw
    # The F411 path crosses TCP -> ROS -> USB CDC -> 115200 UART. Smaller
    # packets reduce worst-case blocking and USB/UART burst pressure while the
    # bootloader's idempotent writes make retries safe.
    step=128 if link.args.transport=='f411' else 192
    for off in range(0,len(staged),step):
        chunk=staged[off:off+step]
        request=bytes((COMM_WRITE_NEW_APP_DATA,))+struct.pack('>I',off)+chunk
        last_error=None
        for attempt in range(1,5):
            try:
                p=link.transact(request,COMM_WRITE_NEW_APP_DATA,2.5)
                if len(p)>=6 and p[1]==1 and struct.unpack('>I',p[2:6])[0]==off:
                    last_error=None; break
                last_error=RuntimeError(f'bad write ACK at {off}: {p.hex()}')
            except Exception as e:
                last_error=e
                if link.f411_direct and 'OWNER:RUNTIME' in str(e):
                    try:
                        link._refresh_f411_maintenance(force=True)
                        print(f'[F411] maintenance ownership restored at offset={off}', flush=True)
                    except Exception as me:
                        last_error=RuntimeError(f'{e}; maintenance restore failed: {me}')
            if attempt<4:
                print(f'[VESC] retry offset={off} attempt={attempt+1} reason={last_error}', flush=True)
                # One immediate idempotent retry handles a lost ACK. If two
                # attempts fail on TCP, rebuild the socket/maintenance route.
                if attempt==2 and link.sock is not None:
                    _recover_bootloader_transport(link,f'offset={off}')
                time.sleep(.08*attempt)
        if last_error is not None:
            raise RuntimeError(f'write failed at {off}: {last_error}')
        if off==0 or off+len(chunk)>=len(staged) or off%(step*40)==0:
            print(f'[VESC] session={session} write {min(off+len(chunk),len(staged))}/{len(staged)}', flush=True)
        time.sleep(.001)


def upload(link,fw:bytes):
    if not fw or len(fw)>MAX_FW: raise RuntimeError(f'firmware size {len(fw)} exceeds {MAX_FW}')
    hw=None; last_error=None
    # An interrupted resident-bootloader staging/copy can leave the F103 busy for
    # tens of seconds before the application services USART3 again. Keep the
    # F411 maintenance ownership alive and probe gently instead of failing after
    # a fixed handful of long transactions.
    probe_deadline=time.monotonic()+75.0
    attempt=0
    while time.monotonic()<probe_deadline:
        attempt += 1
        try:
            hw=fw_version(link,1.2)
            break
        except Exception as e:
            last_error=e
            if link.sock is not None and link.args.transport=='tcp':
                try:
                    link.reconnect_tcp()
                except Exception as re:
                    last_error=re
            if attempt==1 or attempt%4==0:
                remain=max(0,int(probe_deadline-time.monotonic()))
                print(f'[VESC] waiting for F103 response attempt={attempt} remaining={remain}s',flush=True)
            time.sleep(.15)
    if hw is None: raise RuntimeError(f'initial firmware probe failed after recovery window: {last_error}')
    wait_for_bootloader(link,hw)

    stage_error=None
    for session in range(1,4):
        try:
            _stage_once(link,fw,session)
            stage_error=None
            break
        except RestartUploadSession as e:
            stage_error=e
        except Exception as e:
            stage_error=e
        if session>=3: break
        print(f'[VESC] staging session {session} failed: {stage_error}; restarting from erase', flush=True)
        _recover_bootloader_transport(link,f'restart staging session {session+1}')
    if stage_error is not None:
        raise RuntimeError(f'staging failed after recovery attempts: {stage_error}')

    link.write(frame(bytes((COMM_JUMP_TO_BOOTLOADER,))))
    print('[VESC] staged CRC complete; bootloader copy requested', flush=True)
    link.buf.clear()
    if hasattr(link, 'linebuf'): link.linebuf.clear()
    deadline=time.monotonic()+45
    last=''; stable_app_probes=0; probe_failures=0
    while time.monotonic()<deadline:
        time.sleep(.4)
        try:
            last=fw_version(link,1.5)
            probe_failures=0
            if last and 'bootloader' not in last.lower():
                stable_app_probes += 1
                if stable_app_probes >= 2:
                    print(f'[VESC] application returned stable: {last}', flush=True); return
            else:
                stable_app_probes = 0
        except Exception as e:
            stable_app_probes = 0; probe_failures += 1
            if probe_failures>=3 and link.sock is not None:
                try:
                    link.reconnect_tcp(); probe_failures=0
                    print(f'[VESC] reconnect while waiting for updated application: {e}', flush=True)
                except Exception:
                    pass
    raise RuntimeError(f'application did not return stably after update; last={last!r}')

def selftest():
    p=b'\x00\x06\x00test\x00'; f=frame(p)
    assert f[0]==2 and f[1]==len(p) and f[-1]==3
    assert crc16(b'123456789')==0x31C3
    print('PIO_VESC_UPLOADER_SELFTEST_PASS')

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--transport',choices=['serial','tcp','f411'])
    ap.add_argument('--serial-port'); ap.add_argument('--baud',type=int,default=1000000)
    ap.add_argument('--host',default='127.0.0.1'); ap.add_argument('--port',type=int,default=65101)
    ap.add_argument('--firmware'); ap.add_argument('--selftest',action='store_true')
    a=ap.parse_args()
    if a.selftest: selftest(); return
    if not a.transport or not a.firmware: ap.error('--transport and --firmware required')
    if a.transport in ('serial','f411') and not a.serial_port: ap.error('--serial-port required')
    fw=Path(a.firmware).read_bytes()
    lock_key=f'{a.transport}_{a.host}_{a.port}_{a.serial_port or "none"}'
    with UploadProcessLock(lock_key):
        link=Link(a)
        try: upload(link,fw)
        finally: link.close()
if __name__=='__main__':
    try: main()
    except Exception as e:
        print(f'UPLOAD_FAIL: {e}',file=sys.stderr); raise SystemExit(2)
