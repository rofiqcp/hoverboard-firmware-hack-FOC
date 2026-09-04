#!/usr/bin/env python3
from pathlib import Path
import subprocess,tempfile,shutil,sys
R=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
src=[R/'tools/tests/host/test_encoder_abi_runtime.c',R/'Src/motor/mcpwm_foc.c',
     R/'Src/motor/foc_math.c',R/'Src/encoder/encoder.c',
     R/'Src/encoder/enc_abi.c',R/'Src/encoder/encoder_cfg.c']
for cc in [c for c in ('gcc','clang') if shutil.which(c)]:
    with tempfile.TemporaryDirectory(prefix='encoder-abi-') as td:
        exe=Path(td)/'t'
        cmd=[cc,'-std=c11','-O2','-Wall','-Wextra','-Werror',f'-I{R}',f'-I{R/"Src"}',
             f'-I{R/"tools/support/host_stubs"}',*map(str,src),'-lm','-o',str(exe)]
        x=subprocess.run(cmd,text=True,capture_output=True)
        if x.returncode: print(x.stdout+x.stderr);sys.exit(x.returncode)
        x=subprocess.run([str(exe)],text=True,capture_output=True)
        if x.returncode: print(x.stdout+x.stderr);sys.exit(x.returncode)
        print(cc,x.stdout.strip())
print('ENCODER_ABI_RUNTIME_GCC_CLANG_PASS')
