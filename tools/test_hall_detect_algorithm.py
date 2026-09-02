#!/usr/bin/env python3
from pathlib import Path
import subprocess,tempfile,shutil,sys
R=Path(__file__).resolve().parents[1]
for cc in [c for c in ('gcc','clang') if shutil.which(c)]:
    with tempfile.TemporaryDirectory(prefix='hall-v14-') as td:
        exe=Path(td)/'t'
        cmd=[cc,'-std=c11','-O2','-Wall','-Wextra','-Werror',f'-I{R}',f'-I{R/"Src"}',f'-I{R/"tools/hall_detect_stubs"}',
             str(R/'tools/test_hall_detect_algorithm.c'),str(R/'Src/motor/mcpwm_foc.c'),str(R/'Src/motor/foc_math.c'),'-lm','-o',str(exe)]
        x=subprocess.run(cmd,text=True,capture_output=True)
        if x.returncode:
            print(x.stdout+x.stderr);sys.exit(x.returncode)
        x=subprocess.run([str(exe)],text=True,capture_output=True)
        if x.returncode:
            print(x.stdout+x.stderr);sys.exit(x.returncode)
        print(cc,x.stdout.strip())
print('HALL_DETECT_ALGORITHM_GCC_CLANG_PASS')
