#!/usr/bin/env python3
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[3]
ELF = ROOT / '.pio/build/APP_F411/firmware.elf'
OBJDUMP = shutil.which('arm-none-eabi-objdump')
if not OBJDUMP:
    candidate = Path.home() / '.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-objdump'
    if candidate.exists():
        OBJDUMP = str(candidate)

if not OBJDUMP:
    raise SystemExit('FAIL arm-none-eabi-objdump not found')
if not ELF.exists():
    raise SystemExit('FAIL firmware.elf missing; run pio run -e APP_F411 first')

text = subprocess.check_output([OBJDUMP, '-d', '-C', str(ELF)], text=True)
graph = {}
current = None
for line in text.splitlines():
    m = re.match(r'^([0-9a-f]+) <([^>]+)>:', line)
    if m:
        current = m.group(2)
        graph.setdefault(current, set())
        continue
    if current:
        m = re.search(r'\bbl(?:x)?\b[^<]*<([^>]+)>', line)
        if m:
            graph[current].add(m.group(1).split('+')[0])

root = 'DMA1_Channel1_IRQHandler'
if root not in graph:
    raise SystemExit(f'FAIL {root} not found in ELF')

reachable = set()
stack = [root]
while stack:
    fn = stack.pop()
    if fn in reachable:
        continue
    reachable.add(fn)
    stack.extend(graph.get(fn, ()))

forbidden_exact = {
    '__aeabi_ldivmod', '__aeabi_uldivmod',
    'sqrtf', 'sqrt', 'atan2f', 'atan2', 'sinf', 'cosf',
}
forbidden = []
for fn in sorted(reachable):
    if fn in forbidden_exact or fn.startswith('__aeabi_f') or fn.startswith('__aeabi_d'):
        forbidden.append(fn)

if forbidden:
    raise SystemExit('FAIL forbidden ISR calls: ' + ', '.join(forbidden))
# ARMv7-M SDIV/UDIV are hardware instructions and are allowed. The F103 ISR
# budget only forbids software float/double and 64-bit division helpers.
print(
    'ISR_CALLGRAPH_PASS',
    f'reachable={len(reachable)}',
    'forbidden=0',
)
