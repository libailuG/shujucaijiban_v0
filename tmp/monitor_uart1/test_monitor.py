import sys,struct,re,math
from pathlib import Path
sys.path.insert(0,str(Path('tmp/protocol_test_deps').resolve()))
from unicorn import *
from unicorn.arm_const import *
elf=Path(r'C:/Users/libai/Desktop/X1/imu_caiji/zhuban_caiji/ObjectsCli/zhuban_led.axf').read_bytes()
text=Path('tmp/monitor_uart1/symbols.txt').read_text(encoding='utf-8-sig')
def sym(n):
 m=re.search(r'^\s*\d+\s+'+re.escape(n)+r'\s+(0x[0-9a-fA-F]+)',text,re.M);assert m,n;return int(m[1],16)
u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS);u.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M7)
for a,n in [(0x08000000,0x100000),(0x20000000,0x10000),(0x40000000,0x20000),(0x58020000,0x10000),(0xe000e000,0x2000)]:u.mem_map(a,n)
p=struct.unpack_from('<I',elf,28)[0];sz,n=struct.unpack_from('<HH',elf,42)
for i in range(n):
 t,off,va,pa,fs,ms,flags,align=struct.unpack_from('<8I',elf,p+i*sz)
 if t==1 and fs:u.mem_write(va,elf[off:off+fs])
u.reg_write(UC_ARM_REG_C1_C0_2,0xf00000);u.reg_write(UC_ARM_REG_FPEXC,0x40000000)
w=lambda a,v:u.mem_write(a,struct.pack('<I',v&0xffffffff));r=lambda a:struct.unpack('<I',u.mem_read(a,4))[0]
def call(n,a=0,b=0,c=0,d=0):
 u.reg_write(UC_ARM_REG_SP,0x20007e00);u.reg_write(UC_ARM_REG_LR,0x080ff001);u.reg_write(UC_ARM_REG_R0,a);u.reg_write(UC_ARM_REG_R1,b);u.reg_write(UC_ARM_REG_R2,c);u.reg_write(UC_ARM_REG_R3,d)
 u.emu_start(sym(n),0x080ff000,count=20000000)
def command(flags,l=0.,rr=0.):
 for reg,v in [(UC_ARM_REG_S0,l),(UC_ARM_REG_S1,rr)]:u.reg_write(reg,struct.unpack('<I',struct.pack('<f',v))[0])
 call('motor_debug_receive_command',1 if flags&1 else 0,1 if flags&2 else 0,struct.unpack('<I',struct.pack('<f',l))[0],struct.unpack('<I',struct.pack('<f',rr))[0])
def val(n):return r(sym(n))

def value(n):return r(sym(n))
def f(a,v):u.mem_write(a,struct.pack('<f',v))
def floats(a,n):return struct.unpack('<'+'f'*n,u.mem_read(a,4*n))
def seed(t,l=.2,rr=-.3):
 w(0x40000024,t)
 a=sym('g_imu_body_data');u.mem_write(a,bytes(56))
 w(a+36,t);w(a+44,1);w(a+48,1)
 for n,angle in [('g_motor_left',l),('g_motor_right',rr)]:
  a=sym(n);w(a+48+8,1);f(a+48+20,angle);w(a+108,t);w(a+112,1)
def poll(t):seed(t);call('control_mode1_poll')
def get():
 call('control_mode1_get_command',0x2000c000)
 return floats(0x2000c000,2)
u.mem_write(0x20000000,bytes(0x10000))
call('reserved_uart1_init')
w(sym('g_uart2_debug_mode'),0)
call('reserved_uart1_poll'); assert val('g_uart1_tx_count')==0
w(sym('g_uart2_debug_mode'),3)
def drain():
 data=bytearray()
 for i in range(160):
  w(0x40004c1c,128);call('UART4_IRQHandler');data.append(r(0x40004c28)&255)
 w(0x40004c1c,64);call('UART4_IRQHandler')
 assert r(0x40004c00)&(128|64)==0
 return bytes(data)
def check_packet(status):
 m=bytes(u.mem_read(sym('g_control_mode1_monitor'),160))
 assert struct.unpack_from('<III',m)==(0xaaaaaaaa,1,160)
 assert struct.unpack_from('<I',m,156)[0]==0xeeeeeeee
 assert struct.unpack_from('<i',m,20)[0]==status
 packet=drain();assert packet==m
 return m
poll(100000)
for i in range(1,12):
 poll(100000+i*20000)
 m=check_packet(1 if i<=10 else 0)
 flags=struct.unpack_from('<I',m,24)[0]
 assert flags==(15 if i<=10 else 63),flags
 obs=floats(sym('output'),65)
 monitored=struct.unpack_from('<11f',m,80)
 assert all(abs(a-b)<1e-6 for a,b in zip(monitored,obs[54:65]))
 assert all(abs(a-b)<1e-6 for a,b in zip(struct.unpack_from('<2f',m,148),get()))
assert val('g_uart1_tx_count')==11
# Busy sender must not overwrite an in-flight frame.
poll(340000); old=bytes(u.mem_read(sym('g_control_mode1_monitor'),160))
poll(360000); assert val('g_uart1_tx_drop_count')==1
assert drain()==old
# Fault report clears action/command, invalidates obs; repeated faults throttle.
w(sym('g_motor_right')+108,0);w(0x40000024,380000);call('control_mode1_poll')
m=check_packet(-1);assert m[80:156]==bytes(76)
seq=r(sym('g_control_mode1_monitor')+12)
for t in [380001,390000,399999]:
 w(0x40000024,t);call('control_mode1_poll')
 assert r(sym('g_control_mode1_monitor')+12)==seq
# Outside mode3 no frames, including the removed test message.
w(sym('g_uart2_debug_mode'),0);call('control_mode1_poll')
for t in [1000000,2000000]:w(0x40000024,t);call('reserved_uart1_poll')
assert r(0x40004c00)&(128|64)==0
print('PASS ARM: 160B framing, 50Hz warmup/inference, latest11 match obs54..64, final commands, IRQ byte stream, busy drop, zero fault frame, fault throttle, silence outside mode3')