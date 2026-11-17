"""Execute compiled Cortex-M7 functions; peripheral events are injected, not hardware emulated."""
import sys, re, struct
from pathlib import Path
sys.path.insert(0, str(Path('tmp/protocol_test_deps').resolve()))
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_MEM_READ
from unicorn.arm_const import *
elf=Path('C:/Users/libai/Desktop/X1/imu_caiji/zhuban_caiji/ObjectsCli/zhuban_led.axf').read_bytes()
symbols=Path('tmp/imu_bdma_symbols.txt').read_text(encoding='utf-8-sig')
def sym(n):
    m=re.search(r'^\s*\d+\s+'+re.escape(n)+r'\s+(0x[0-9a-fA-F]+)',symbols,re.M)
    assert m,n
    return int(m[1],16)
def setup():
    global u
    u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
    u.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M7)
    for a,n in [(0x08000000,0x100000),(0x20000000,0x20000),(0x24000000,0x1000),
                (0x38000000,0x1000),(0x40000000,0x20000),(0x58000000,0x30000),(0xe000e000,0x2000)]:
        u.mem_map(a,n)
    p=struct.unpack_from('<I',elf,28)[0]; sz,n=struct.unpack_from('<HH',elf,42)
    for i in range(n):
        t,off,va,pa,fs,ms,flags,align=struct.unpack_from('<8I',elf,p+i*sz)
        if t==1 and fs:u.mem_write(va,elf[off:off+fs])
    # GPIO inputs high, and skip the initialization-only physical bus delays.
    w(0x58020c10,(1<<12)|(1<<13))
    def advance_timer(uc,access,address,size,value,user):
        w(0x40000024,r(0x40000024)+10)
    hook=u.hook_add(UC_HOOK_MEM_READ,advance_timer,begin=0x40000024,end=0x40000027)
    call('imu_bus_init')
    u.hook_del(hook)
def w(a,v):u.mem_write(a,struct.pack('<I',v&0xffffffff))
def r(a):return struct.unpack('<I',u.mem_read(a,4))[0]
def val(n):return r(sym(n))
def call(n,*args):
    u.reg_write(UC_ARM_REG_SP,0x2001f000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
    for reg,v in zip([UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3],args):u.reg_write(reg,v)
    u.emu_start(sym(n),0x080ff000,count=100000)
    assert u.reg_read(UC_ARM_REG_PC)==0x080ff000, 'function did not return: '+n
I=0x58001c00; B=0x58025400; C=B+8
def event(flags):
    w(I+0x18,flags);call('I2C4_EV_IRQHandler');w(I+0x18,0)
def start():
    w(0x40000024,1000);call('imu_bus_schedule_sample',0x68)
    event(2);event(64)
    assert r(C+4)==12 and r(C)&1
    assert r(I)&(1<<15) and not r(I)&(1<<2) # RXDMAEN, no RXIE
def dma_done(payload=bytes(range(12))):
    u.mem_write(sym('sample_bytes'),payload);w(C+4,0);w(B,2)
    call('BDMA_Channel0_IRQHandler');w(B,0)
def take(expected=0):
    call('imu_bus_take_sample',0x2001e000,0x2001e020,0x2001e024)
    assert u.reg_read(UC_ARM_REG_R0)==1
    assert r(0x2001e024)==expected&0xffffffff
    if expected==0:
        assert bytes(u.mem_read(0x2001e000,12))==bytes(range(12))
        assert r(0x2001e020)==1000
    assert val('sample_done')==0 and not r(C)&1
for order in ['dma_first','stop_first']:
    setup();start()
    if order=='dma_first':dma_done();assert not val('sample_done');event(32)
    else:event(32);assert not val('sample_done');dma_done()
    take();print('PASS',order)
setup();start();event(16);take(-11);print('PASS NACK abort')
setup();start();w(B,8);call('BDMA_Channel0_IRQHandler');take(-12);print('PASS BDMA error abort')
setup();start();event(32);w(0x40000024,2000);call('imu_bus_schedule_sample',0x68);take(-10)
print('PASS short receive timeout')
setup();start();call('imu_bus_cancel_sample');w(B,2);call('BDMA_Channel0_IRQHandler')
assert val('sample_active')==0 and val('sample_done')==0 and not r(C)&1
print('PASS cancel and late IRQ')
setup();start();w(C+4,1);w(B,2);call('BDMA_Channel0_IRQHandler');w(B,0);event(32);take(-12)
print('PASS short transfer rejected')
print('7 cases passed (injected register events; no physical I2C/cache validation)')
