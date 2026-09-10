from pathlib import Path
import re
root=Path(r'D:\github\hoverboard-vesc')

def one(s,old,new,label):
    if old not in s: raise SystemExit(label+' not found')
    return s.replace(old,new,1)

def sub1(s,pat,new,label):
    out,n=re.subn(pat,lambda m:new,s,count=1,flags=re.S)
    if n!=1: raise SystemExit(label+' not found')
    return out

up=root/'tools'/'pio_vesc_upload.py'
t=up.read_text(encoding='utf-8-sig')
t=one(t,'import argparse, fcntl, os, signal, socket, struct, subprocess, sys, time\nfrom pathlib import Path\n',
'''import argparse, os, signal, socket, struct, subprocess, sys, tempfile, time
from pathlib import Path
try:
    import fcntl
except ImportError:
    fcntl=None
try:
    import msvcrt
except ImportError:
    msvcrt=None
''','imports')
lock='''class UploadProcessLock:
    def __init__(self,key:str):
        safe=''.join(c if c.isalnum() else '_' for c in key)[:96]
        self.path=Path(tempfile.gettempdir())/f'pio_vesc_upload_{safe}.lock'
        self.fd=None
    def __enter__(self):
        self.fd=os.open(self.path,os.O_CREAT|os.O_RDWR,0o660)
        if os.name=='nt':
            if msvcrt is None: raise RuntimeError('Windows locking unavailable')
            if os.path.getsize(self.path)==0: os.write(self.fd,b'\\0')
            os.lseek(self.fd,0,os.SEEK_SET)
            try: msvcrt.locking(self.fd,msvcrt.LK_NBLCK,1)
            except OSError:
                os.lseek(self.fd,1,os.SEEK_SET)
                owner=os.read(self.fd,160).decode(errors='replace').strip()
                os.close(self.fd); self.fd=None
                raise RuntimeError('another firmware uploader is active'+(f' ({owner})' if owner else ''))
        else:
            if fcntl is None: raise RuntimeError('POSIX locking unavailable')
            try: fcntl.flock(self.fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:
                os.lseek(self.fd,0,os.SEEK_SET)
                owner=os.read(self.fd,160).decode(errors='replace').strip('\\0').strip()
                os.close(self.fd); self.fd=None
                raise RuntimeError('another firmware uploader is active'+(f' ({owner})' if owner else ''))
'''
lock+='''        meta=f'pid={os.getpid()} started={time.time():.3f}'.encode()
        os.ftruncate(self.fd,0)
        os.write(self.fd,b'\\0'+meta if os.name=='nt' else meta)
        os.fsync(self.fd); return self
    def __exit__(self,exc_type,exc,tb):
        if self.fd is None: return
        try:
            if os.name=='nt':
                os.lseek(self.fd,0,os.SEEK_SET); msvcrt.locking(self.fd,msvcrt.LK_UNLCK,1)
            else:
                fcntl.flock(self.fd,fcntl.LOCK_UN)
        finally:
            os.close(self.fd); self.fd=None
'''
t=sub1(t,r'class UploadProcessLock:.*?(?=\ndef crc16)',lock,'lock')
helpers='''
def _stable_linux_port(device:str)->str:
    if os.name!='posix': return device
    root=Path('/dev/serial/by-id')
    try:
        target=os.path.realpath(device)
        for p in sorted(root.iterdir()):
            if os.path.realpath(str(p))==target: return str(p)
    except OSError: pass
    return device

'''
t=one(t,'\nclass Link:',helpers+'class Link:','helper insert')
t=one(t,"        import serial\n        self.ser=serial.Serial(self.args.serial_port,self.args.baud,timeout=.1,write_timeout=2)\n        self.route='serial'\n",
'''        import serial
        self.ser=serial.Serial(self.args.serial_port,self.args.baud,timeout=.1,write_timeout=2)
        try:
            self.ser.reset_input_buffer(); self.ser.reset_output_buffer()
        except Exception: pass
        self.route='serial'
''','serial open')
resolver='''
def _serial_candidates():
    import serial.tools.list_ports
    ranked=[]
    for p in serial.tools.list_ports.comports():
        dev=_stable_linux_port(p.device)
        meta=' '.join(str(x or '') for x in (p.description,p.manufacturer,p.hwid)).lower()
        score=(100 if p.vid is not None else 0)+(40 if 'usb' in meta else 0)
        if 'ch340' in meta or 'ch341' in meta or p.vid==0x1A86: score+=30
        if '/dev/serial/by-id/' in dev: score+=25
        if 'bluetooth' in meta or 'active management' in meta or 'amt' in meta: score-=150
        ranked.append((-score,dev,p.description or ''))
    ranked.sort(key=lambda x:(x[0],x[1])); out=[]
    for _,dev,desc in ranked:
        if dev not in [x[0] for x in out]: out.append((dev,desc))
    return out
'''
resolver+='''
def resolve_serial_port(args):
    requested=(args.serial_port or 'auto').strip()
    if requested.lower() not in ('auto','detect'): return requested
    override=os.environ.get('VESC_SERIAL_PORT','').strip()
    candidates=_serial_candidates()
    if override: candidates=[(override,'VESC_SERIAL_PORT override')]+[x for x in candidates if x[0]!=override]
    if not candidates: raise RuntimeError('no serial ports detected; use --serial-port explicitly')
    errors=[]
    for dev,desc in candidates:
        probe=argparse.Namespace(**vars(args)); probe.serial_port=dev; link=None
        try:
            link=Link(probe); hw=fw_version(link,1.0)
            print(f'[PORT] selected {dev} ({desc}) target={hw}',flush=True); return dev
        except Exception as e:
            errors.append(f'{dev}:{type(e).__name__}')
        finally:
            if link is not None:
                try: link.close()
                except Exception: pass
    raise RuntimeError('no VESC/F103 target responded on serial ports: '+', '.join(errors))

'''
t=one(t,'def wait_for_bootloader(link, initial_hw: str) -> str:\n',resolver+'def wait_for_bootloader(link, initial_hw: str) -> str:\n','resolver')
t=one(t,
"    ap.add_argument('--transport',choices=['serial','tcp','f411'])\n    ap.add_argument('--serial-port'); ap.add_argument('--baud',type=int,default=1000000)\n",
"    ap.add_argument('--transport',choices=['serial','tcp','f411'])\n    ap.add_argument('--serial-port',default='auto',help='serial device, or auto for VESC/F103 probe-based detection')\n    ap.add_argument('--baud',type=int,default=1000000)\n",
'main args')
old="""    ap.add_argument('--tcp-wait',type=float,default=5.0,help='maksimum deteksi TCP maintenance sebelum fallback F411 CDC (maks 5 s)')
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
"""
new="""    ap.add_argument('--tcp-wait',type=float,default=5.0,help='maksimum deteksi TCP maintenance sebelum fallback F411 CDC (maks 5 s)')
    ap.add_argument('--firmware'); ap.add_argument('--selftest',action='store_true'); ap.add_argument('--probe-only',action='store_true')
    a=ap.parse_args()
    if a.selftest: selftest(); return
    if not a.transport: ap.error('--transport required')
"""
new+='''    if a.transport=='serial':
        a.serial_port=resolve_serial_port(a)
    elif a.transport=='f411' and (not a.serial_port or a.serial_port.lower() in ('auto','detect')):
        ap.error('--serial-port must be explicit for f411 transport')
    lock_key=f'{a.transport}_{a.host}_{a.port}_{a.serial_port or "none"}'
    if a.probe_only:
        with UploadProcessLock(lock_key):
            link=Link(a)
            try: print(f'VESC_TARGET_PROBE_PASS port={a.serial_port} hw={fw_version(link,2.0)}',flush=True)
            finally: link.close()
        return
    if not a.firmware: ap.error('--firmware required unless --probe-only or --selftest')
    fw=Path(a.firmware).read_bytes()
    with UploadProcessLock(lock_key):
        link=Link(a)
        try: upload(link,fw)
        finally: link.close()
'''
t=one(t,old,new,'main tail')
up.write_text(t,encoding='utf-8',newline='\n')
pio=root/'platformio.ini'; p=pio.read_text(encoding='utf-8-sig')
p=one(p,'upload_port = /dev/ttyUSB_F103','upload_port = auto','pio port')
pio.write_text(p,encoding='utf-8',newline='\n')

boot=root/'Src'/'bootloader'/'main.c'; s=boot.read_text(encoding='utf-8-sig')
s=one(s,'    g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_NOPULL; g.Pin = GPIO_PIN_11; HAL_GPIO_Init(GPIOB, &g);',
      '    g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP; g.Pin = GPIO_PIN_11; HAL_GPIO_Init(GPIOB, &g);','rx pullup')
s=one(s,'static void uart_init(void) {','static bool uart_init(void) {','uart sig')
s=one(s,'    (void)HAL_UART_Init(&huart3);\n}','    return HAL_UART_Init(&huart3) == HAL_OK;\n}','uart result')
s=one(s,
'''/* OTA staging is erased lazily, one 2-KiB flash page at a time. This avoids
 * a long 120-KiB blocking erase before COMM_ERASE_NEW_APP can ACK and keeps
 * every flash operation bounded. 120 KiB / 2 KiB = 60 pages. */''',
'''/* Recovery update state is explicit and fail-closed. The host erases the
 * complete staging region before a new session; duplicate chunk writes remain
 * idempotent and PENDING metadata is written only after full-image CRC passes. */''','stage comment')
erase='''static bool flash_page_erased(uint32_t address) {
    for (uint32_t off=0u; off<F103_FLASH_PAGE_SIZE; off+=4u) {
        if (*(volatile const uint32_t *)(address+off) != 0xFFFFFFFFu) return false;
    }
    return true;
}

static bool erase_pages(uint32_t base, uint32_t bytes) {
    if ((base & (F103_FLASH_PAGE_SIZE - 1u)) != 0u || bytes == 0u) return false;
    const uint32_t pages=(bytes+F103_FLASH_PAGE_SIZE-1u)/F103_FLASH_PAGE_SIZE;
    for (uint32_t page=0u; page<pages; ++page) {
        const uint32_t address=base+page*F103_FLASH_PAGE_SIZE;
        if (!ram_flash_erase_page(address) || !flash_page_erased(address)) return false;
    }
    return true;
}
'''
s=sub1(s,r'static bool erase_pages\(uint32_t base, uint32_t bytes\) \{.*?\n\}\n(?=\nstatic bool program_halfwords)',erase,'erase pages')
vec='''static bool app_vector_valid_for_size(uint32_t image_size) {
    if (image_size < 8u || image_size > F103_APP_REGION_SIZE) return false;
    const uint32_t sp=*(const uint32_t *)F103_APP_BASE_ADDR;
    const uint32_t rv=*(const uint32_t *)(F103_APP_BASE_ADDR+4u);
    if (sp < 0x20000000u || sp > F103_BOOT_REQUEST_ADDR || (sp & 3u)) return false;
    if ((rv & 1u) == 0u) return false;
    const uint32_t pc=rv & ~1u;
    return pc >= F103_APP_BASE_ADDR && pc < (F103_APP_BASE_ADDR+image_size);
}

static bool app_vector_valid(void) {
    return app_vector_valid_for_size(F103_APP_REGION_SIZE);
}
'''
s=sub1(s,r'static bool app_vector_valid\(void\) \{.*?\n\}\n(?=\nstatic bool meta_valid)',vec,'app vector')
s=one(s,'    if (!app_vector_valid()) { boot_diag_copy_code = 4000u; return false; }',
      '    if (!app_vector_valid_for_size(size)) { boot_diag_copy_code = 4000u; return false; }','copy vector')
meta='''static bool meta_valid(const f103_update_meta_t *m) {
    if (!m || m->magic != F103_UPDATE_META_MAGIC) return false;
    if (m->size != ~m->size_inv) return false;
    if ((uint16_t)(m->crc16 ^ m->crc16_inv) != 0xFFFFu) return false;
    if (m->version != F103_UPDATE_META_VERSION || (uint16_t)(m->version ^ m->version_inv) != 0xFFFFu) return false;
    if (m->state == F103_UPDATE_STATE_PENDING) return m->size > 0u && m->size <= F103_MAX_FW_IMAGE_SIZE;
    if (m->state == F103_UPDATE_STATE_RECOVERY) return m->size == 0u && m->crc16 == 0u;
    return false;
}
'''
s=sub1(s,r'static bool meta_valid\(const f103_update_meta_t \*m\) \{.*?\n\}\n(?=\nstatic bool write_meta)',meta,'meta')
s=one(s,'    safe_gpio_init();\n    uart_init();',
'''    safe_gpio_init();
    if (!uart_init()) {
        for (;;) { HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_2); HAL_Delay(100u); }
    }''','uart failsafe')
boot.write_text(s,encoding='utf-8',newline='\n')
print('PATCH_BOOT_ROBUST_DONE')

