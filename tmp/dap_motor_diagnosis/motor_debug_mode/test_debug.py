import sys,struct,math
from pathlib import Path
sys.path.insert(0,str(Path('tmp/protocol_test_deps').resolve()))
from unicorn import *
from unicorn.arm_const import *
elf=Path(r'C:/Users/libai/Desktop/X1/imu_caiji/zhuban_caiji/Objects/zhuban_led.axf').read_bytes()
u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS);u.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M7)
for a,n in [(0x08000000,0x20000),(0x20000000,0x10000),(0x40000000,0x20000),(0x58020000,0x10000),(0xe000e000,0x2000)]:u.mem_map(a,n)
p=struct.unpack_from('<I',elf,28)[0];sz,n=struct.unpack_from('<HH',elf,42)
for i in range(n):
 t,off,va,pa,fs,ms,flags,align=struct.unpack_from('<8I',elf,p+i*sz)
 if t==1 and fs:u.mem_write(va,elf[off:off+fs])
u.reg_write(UC_ARM_REG_C1_C0_2,0xf00000);u.reg_write(UC_ARM_REG_FPEXC,0x40000000)
w=lambda a,v:u.mem_write(a,struct.pack('<I',v&0xffffffff))
r=lambda a:struct.unpack('<I',u.mem_read(a,4))[0]
wf=lambda a,v:u.mem_write(a,struct.pack('<f',v))
rf=lambda a:struct.unpack('<f',u.mem_read(a,4))[0]
# SRAM-only test state, no peripheral IRQs or hardware are accessed.
u.mem_write(0x20000000,bytes(0x10000))
def call(addr,a=0,b=0):
 u.reg_write(UC_ARM_REG_SP,0x20007e00);u.reg_write(UC_ARM_REG_LR,0x0801f001)
 u.reg_write(UC_ARM_REG_R0,a);u.reg_write(UC_ARM_REG_R1,b)
 u.emu_start(addr,0x0801f000,count=300000)
def feed(flags,l=0.,rr=0.,tail=0xeeeeeeee):
 packet=struct.pack('<III8fI',0xaaaaaaaa,flags,123,0.,0.,0.,0.,0.,l,rr,0.,tail)
 h=r(0x200008d0)
 for x in packet:u.mem_write(0x200008d4+h,bytes([x]));h=(h+1)%1024
 w(0x200008d0,h)
def poll(now):
 w(0x40000024,now);w(0x20000d8c,0);call(0x08003db1)
 return struct.unpack('<III8fI',bytes(u.mem_read(0x20000d5c,48)))
for base,ang,vel,tq in [(0x20000664,1.25,3.25,.1),(0x2000077c,2.5,4.5,-.2)]:
 w(base+112,1);wf(base+48+20,ang);wf(base+48+16,vel);wf(base+48+12,tq)
wf(0x200005c0+4,15.5);wf(0x200005e4,.3);wf(0x200005e4+4,-.4)
w(0x2000085c+4,0xa0);w(0x2000085c+8,77);wf(0x2000085c+40,10.5)
a=poll(50000);assert a[1]==0xa0 and abs(a[4]-.3)<1e-6 and a[6:10]==(0.,0.,0.,0.)
feed(1,.75,-.5);a=poll(100000)
assert r(0x2000061c)==1 and rf(0x20000624)==.75 and rf(0x20000628)==-.5
assert a[1]==0xa1 and a[2]==77 and a[3]==15.5 and a[4:8]==(1.25,2.5,3.25,4.5)
assert abs(a[8]-.1)<1e-6 and abs(a[9]+.2)<1e-6 and a[10]==10.5
# No new messages for 10s: mode and last torques remain active by explicit request.
a=poll(10100000);assert a[1]&1 and rf(0x20000624)==.75
# Check real motor frame builder for both sides, mode1 and KP5, no other controls.
for obj,base,mid,tq,uart,gpio,de in [(0x2000000c,0x20000664,126,.75,0x40004800,0x58020c00,1<<10),(0x200002ac,0x2000077c,127,-.5,0x40011400,0x58020800,1<<8)]:
 for off,val in [(0,uart),(4,gpio),(8,de),(12,mid),(16,base),(592,0)]:w(obj+off,val)
 call(0x08004201,obj,10100000)
 f=struct.unpack('<III7fII',bytes(u.mem_read(base,48)))
 assert f[:3]==(0xaaaaaaaa,mid,1) and f[3]==tq and f[4:8]==(0.,0.,0.,0.) and f[8]==5. and f[9]==0. and f[10:]==(0,0xeeeeeeee)
# Bad tail cannot enter command state; nonfinite open command rejected and not applied.
feed(0,tail=0);poll(10150000);assert r(0x2000061c)==1
feed(1,float('nan'),1.);poll(10200000);assert r(0x20000620)==1 and rf(0x20000624)==.75
# Explicit OFF accepts even unused nonfinite torque fields and clears mode/targets.
feed(0,float('nan'),float('inf'));a=poll(10250000)
assert r(0x2000061c)==0 and rf(0x20000624)==0. and rf(0x20000628)==0. and a[1]==0xa0
assert a[6:10]==(0.,0.,0.,0.) and abs(a[4]-.3)<1e-6 and abs(a[5]+.4)<1e-6
w(0x2000000c+592,0);call(0x08004201,0x2000000c,10250000)
f=struct.unpack('<III7fII',bytes(u.mem_read(0x20000664,48)));assert f[2]==1 and all(x==0 for x in f[3:11])
print('PASS actual Keil ARM: switch1 enter/ACK, paired feedback units/order, signed torque routing KP5,10s silence hold,bad-tail/NaN rejection,explicit OFF mode1 zero/KP0, normal Roll/Pitch restore,unrelated fields preserved')
