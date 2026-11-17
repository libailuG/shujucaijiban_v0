import sys,struct
from pathlib import Path
sys.path.insert(0,str(Path('tmp/protocol_test_deps').resolve()))
from unicorn import *
from unicorn.arm_const import *
elf=Path(r'C:/Users/libai/Desktop/X1/motor/motor_G431_new_protocol/motor_G431_new_protocol/motor/MDK-ARM/taixi_motor_driver1/taixi_motor_driver1.axf').read_bytes()
u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS);u.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
u.mem_map(0x08000000,0x20000);u.mem_map(0x20000000,0x10000)
phoff=struct.unpack_from('<I',elf,28)[0];sz,n=struct.unpack_from('<HH',elf,42)
for i in range(n):
 t,off,va,pa,fs,ms,flags,align=struct.unpack_from('<8I',elf,phoff+i*sz)
 if t==1 and fs:u.mem_write(va,elf[off:off+fs])
head,tail,ring,epoch=0x200009f8,0x20000bfc,0x200009fc,0x200009f4
w=lambda a,v:u.mem_write(a,struct.pack('<I',v))
r=lambda a:struct.unpack('<I',u.mem_read(a,4))[0]
def reset(pos=0):
 u.mem_write(0x20000000,bytes(0x10000));w(head,pos);w(tail,pos)
def feed(b):
 for x in b:
  h=r(head);assert (h+1)%512!=r(tail);u.mem_write(ring+h,bytes([x]));w(head,(h+1)%512)
def poll():
 u.reg_write(UC_ARM_REG_SP,0x20007e00);u.reg_write(UC_ARM_REG_LR,0x0801f001)
 u.reg_write(UC_ARM_REG_R0,0x20007000);u.reg_write(UC_ARM_REG_R1,0x20007100)
 u.emu_start(0x08008851,0x0801f000,count=200000)
 return bytes(u.mem_read(0x20007000,struct.unpack('<H',u.mem_read(0x20007100,2))[0])) if u.reg_read(UC_ARM_REG_R0) else None
f=struct.pack('<III7fII',0xaaaaaaaa,126,0,*([0.0]*7),0,0xeeeeeeee)
for split in range(49):
 reset();feed(f[:split]);got=poll()
 if split==48:assert got==f
 else:assert got is None;feed(f[split:]);assert poll()==f
reset();feed(b'\x00\xff'+f+f);assert poll()==f;assert poll()==f;assert poll() is None
reset(500);feed(f);assert poll()==f
reset();feed(f[:20]);assert poll() is None;w(epoch,1);assert poll() is None;feed(f);assert poll()==f
reset();bad=f[:-4]+b'\x00'*4;feed(bad+f);assert poll()==f
reset();ota=b'\xaa\x55\x01\x7e\x03\x01\x02\x03\x00\x00';feed(ota[:3]);assert poll() is None;feed(ota[3:]+f);assert poll()==ota;assert poll()==f
print('PASS: actual linked ARM parser; all49 splits, noise, concatenation, ring wrap, error epoch, bad tail resync, OTA split and routing')
