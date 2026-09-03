#!/usr/bin/env python3
from pathlib import Path
import subprocess,tempfile,shutil,sys
R=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
for cc in [c for c in ('gcc','clang') if shutil.which(c)]:
  with tempfile.TemporaryDirectory() as td:
    exe=Path(td)/'t'
    cmd=[cc,'-std=c11','-O2','-Wall','-Wextra','-Werror',f'-I{R}',f'-I{R/"Src"}',f'-I{R/"tools/support/hall_detect_stubs"}',str(R/'tools/tests/host/test_foc_math.c'),str(R/'Src/motor/foc_math.c'),'-o',str(exe)]
    r=subprocess.run(cmd,text=True,capture_output=True)
    if r.returncode: print(r.stdout+r.stderr);sys.exit(r.returncode)
    r=subprocess.run([str(exe)],text=True,capture_output=True)
    if r.returncode: print(r.stdout+r.stderr);sys.exit(r.returncode)
    print(cc,r.stdout.strip())
print('FOC_GCC_CLANG_RUNTIME_PASS')
