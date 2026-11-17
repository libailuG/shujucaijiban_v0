import sys,struct
from pathlib import Path
sys.path.insert(0,str(Path('tmp/protocol_test_deps').resolve()))
from unicorn import *
from unicorn.arm_const import *
elf=Path(r'C:/Users/libai/Desktop/X1/imu_caiji/zhuban_caiji/Objects/zhuban_led.axf').read_bytes()
u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS);u.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M7)
for a,n in [(0x08000000,0x20000),(0x20000000,0x10000),(0x40000000,0x1000),(0x58021000,0x1000)]:u.mem_map(a,n)
p=struct.unpack_from('<I',elf,28)[0];sz,n=struct.unpack_from('<HH',elf,42)
for i in range(n):
 t,off,va,pa,fs,ms,flags,align=struct.unpack_from('<8I',elf,p+i*sz)
 if t==1 and fs:u.mem_write(va,elf[off:off+fs])
w=lambda a,v:u.mem_write(a,struct.pack('<I',v&0xffffffff))
r=lambda a:struct.unpack('<I',u.mem_read(a,4))[0]
ch=[(0x20000660+100,0x20000660+108,0x20000660+112),(0x20000778+100,0x20000778+108,0x20000778+112),(0x200005e4+40,0x200005e4+36,0x200005e4+44),(0x20000844,0x2000084c,0x20000850)]
def reset():u.mem_write(0x20000000,bytes(0x10000))
def sample(i,t,count):
 a,b,c=ch[i];w(a,count);w(b,t);w(c,1)
def poll(t):
 w(0x40000024,t);u.reg_write(UC_ARM_REG_SP,0x20007e00);u.reg_write(UC_ARM_REG_LR,0x0801f001)
 u.emu_start(0x08002609,0x0801f000,count=50000)
 return r(0x20000620),r(0x2000061c),r(0x20000624)
reset();events=[];prev=None
for ms in range(5701):
 sample(0,ms*1000,ms+1);sample(2,ms*1000,ms+1)
 phase,on,mask=poll(ms*1000);assert mask==5
 if phase!=prev:events.append((ms,phase,on));prev=phase
expected=[(0,0,1),(100,1,0),(200,2,1),(300,3,0),(400,4,1),(500,5,0),(1500,6,1),(2000,7,0),(2300,8,1),(2400,9,0),(2700,10,1),(3200,11,0),(3500,12,1),(3600,13,0),(5600,0,1),(5700,1,0)]
assert events==expected,(events,expected)
reset()
for i in range(4):sample(i,0,1)
assert poll(0)[2]==15
for t,mask in [(99999,15),(100000,11),(200000,8),(500000,0),(0x100000000,0)]:assert poll(t)[2]==mask
reset();start=0xffff0000;sample(0,start,1);assert poll(start)[2]==1;assert poll(start+199999)[2]==1;assert poll(start+200000)[2]==0
reset();w(ch[0][2],1);assert poll(0)[2]==0 # sticky-valid alone is insufficient
print('PASS actual Keil ARM: 3-flash marker, channel order, 100/500ms status, 300ms gaps, 1s/2s pauses, per-channel timeout, timer wrap, no-data invalidity')
