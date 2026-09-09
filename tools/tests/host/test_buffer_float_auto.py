#!/usr/bin/env python3
from pathlib import Path
import shutil,subprocess,sys,tempfile
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
for cc in [c for c in ('gcc','clang') if shutil.which(c)]:
    with tempfile.TemporaryDirectory() as td:
        exe=Path(td)/'t'
        cmd=[cc,'-std=c11','-O2','-Wall','-Wextra','-Werror',f'-I{ROOT}/Src',
             str(ROOT/'tools/tests/host/test_buffer_float_auto.c'),str(ROOT/'Src/vesc/buffer.c'),'-lm','-o',str(exe)]
        r=subprocess.run(cmd,text=True,capture_output=True)
        if r.returncode: print(r.stdout+r.stderr);sys.exit(r.returncode)
        r=subprocess.run([str(exe)],text=True,capture_output=True)
        if r.returncode: print(r.stdout+r.stderr);sys.exit(r.returncode)
        print(cc,r.stdout.strip())
print('BUFFER_FLOAT_AUTO_GCC_CLANG_PASS')
