"""Execute the actual Keil-compiled parser tests on an emulated ARM CPU."""
import struct
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent / 'protocol_test_deps'))
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_PROT_ALL
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_CPU_ARM_CORTEX_M7, UC_ARM_REG_PC

elf = (Path(__file__).parent / 'Objects/protocol_test.axf').read_bytes()
assert elf[:6] == b'\x7fELF\x01\x01'
entry, phoff = struct.unpack_from('<II', elf, 24)
phsize, phnum = struct.unpack_from('<HH', elf, 42)
uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
uc.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M7)
uc.mem_map(0x10000, 0x10000, UC_PROT_ALL)
uc.mem_map(0x20000000, 0x20000, UC_PROT_ALL)
for i in range(phnum):
    typ, off, va, pa, size, memsize, flags, align = struct.unpack_from('<8I', elf, phoff + i * phsize)
    if typ == 1 and size:
        uc.mem_write(va, elf[off:off + size])
stop = 0x1F000
uc.reg_write(UC_ARM_REG_SP, 0x2001F000)
uc.reg_write(UC_ARM_REG_LR, stop | 1)
try:
    uc.emu_start(entry | 1, stop, count=5000000)
except Exception:
    pc = uc.reg_read(UC_ARM_REG_PC)
    print(f'PC={pc:#x}, bytes={bytes(uc.mem_read(pc, 8)).hex()}')
    raise
result = uc.reg_read(UC_ARM_REG_R0)
if result:
    raise SystemExit(f'Protocol test FAILED at C line {result}')
print('PASS: motor stop golden bytes, unaligned codec, payload words, direction rejection, all splits, repeated headers, bad tail recovery, consecutive frames')
