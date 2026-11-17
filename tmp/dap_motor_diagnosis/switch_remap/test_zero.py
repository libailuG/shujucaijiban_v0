import sys,struct,re,math
from pathlib import Path
sys.path.insert(0,str(Path('tmp/protocol_test_deps').resolve()))
from unicorn import *
from unicorn.arm_const import *
elf=Path(r'C:/Users/libai/Desktop/X1/imu_caiji/zhuban_caiji/Objects/zhuban_led.axf').read_bytes()
text=Path('tmp/dap_motor_diagnosis/switch_remap/symbols.txt').read_text(encoding='utf-8-sig')
def sym(n):
 m=re.search(r'^\s*\d+\s+'+re.escape(n)+r'\s+(0x[0-9a-fA-F]+)',text,re.M);assert m,n;return int(m[1],16)
u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS);u.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M7)
for a,n in [(0x08000000,0x20000),(0x20000000,0x10000),(0x40000000,0x20000),(0x58020000,0x10000),(0xe000e000,0x2000)]:u.mem_map(a,n)
p=struct.unpack_from('<I',elf,28)[0];sz,n=struct.unpack_from('<HH',elf,42)
for i in range(n):
 t,off,va,pa,fs,ms,flags,align=struct.unpack_from('<8I',elf,p+i*sz)
 if t==1 and fs:u.mem_write(va,elf[off:off+fs])
u.reg_write(UC_ARM_REG_C1_C0_2,0xf00000);u.reg_write(UC_ARM_REG_FPEXC,0x40000000)
w=lambda a,v:u.mem_write(a,struct.pack('<I',v&0xffffffff));r=lambda a:struct.unpack('<I',u.mem_read(a,4))[0]
def call(n,a=0,b=0,c=0,d=0):
 u.reg_write(UC_ARM_REG_SP,0x20007e00);u.reg_write(UC_ARM_REG_LR,0x0801f001);u.reg_write(UC_ARM_REG_R0,a);u.reg_write(UC_ARM_REG_R1,b);u.reg_write(UC_ARM_REG_R2,c);u.reg_write(UC_ARM_REG_R3,d)
 u.emu_start(sym(n),0x0801f000,count=300000)
def command(flags,l=0.,rr=0.):
 for reg,v in [(UC_ARM_REG_S0,l),(UC_ARM_REG_S1,rr)]:u.reg_write(reg,struct.unpack('<I',struct.pack('<f',v))[0])
 call('motor_debug_receive_command',1 if flags&1 else 0,1 if flags&2 else 0,struct.unpack('<I',struct.pack('<f',l))[0],struct.unpack('<I',struct.pack('<f',rr))[0])
def val(n):return r(sym(n))
objs=[sym('left'),sym('right')];data=[sym('g_motor_left'),sym('g_motor_right')];uart=[0x40004800,0x40011400]
now=0;events=[];pending=[None,None];feedback=[];allow=[False,False]
def reset(start=0):
 global now,events,pending,feedback,allow
 u.mem_write(0x20000000,bytes(0x10000));now=start;events=[];pending=[None,None];feedback=[];allow=[False,False]
 for i in range(2):
  for off,v in [(0,uart[i]),(4,0x58020c00 if i==0 else 0x58020800),(8,1<<10 if i==0 else 1<<8),(12,126+i),(16,data[i])]:w(objs[i]+off,v)
 w(0x40000024,now)
def tick(t):
 global now
 now=t;w(0x40000024,now)
 for i in range(2):
  if pending[i] and pending[i][0]<=t:
   _,frame,start=pending[i];w(objs[i]+592,2);w(uart[i],0x6d);w(uart[i]+0x1c,0x40)
   call('USART3_IRQHandler' if i==0 else 'USART6_IRQHandler');w(uart[i]+0x1c,0)
   events.append(('tc',i,frame,t));pending[i]=None
   if frame[2]==1 and allow[i]:feedback.append((t+100,i))
 for due,i in feedback[:]:
  if due<=t:
   w(data[i]+100,r(data[i]+100)+1);w(data[i]+48+8,1);w(data[i]+108,t);w(data[i]+112,1);feedback.remove((due,i))
 call('motor_bus_poll')
 for i in range(2):
  if r(objs[i]+592) and pending[i] is None:
   f=struct.unpack('<III7fII',bytes(u.mem_read(data[i],48)));events.append(('start',i,f,t));pending[i]=(t+250,f,t)
def run(start,end):
 for t in range(start,end+1,100):tick(t)
def zeros():return [e for e in events if e[0]=='start' and e[2][2]==2]
reset();command(1,.6,-.4);run(0,20000)
command(3,3.,-3.);run(20100,100000)
assert len(zeros())==2 and [e[1] for e in zeros()]==[0,1]
lefttc=next(e[3] for e in events if e[0]=='tc' and e[1]==0 and e[2][2]==2)
assert zeros()[1][3]-lefttc>=10000
assert val('g_motor_zero_state')==5 and val('g_motor_zero_recovered')==0
assert val('g_motor_zero_left_sent')==val('g_motor_zero_right_sent')==1
for e in events:
 if e[0]=='start' and e[3]>=20100:assert all(x==0 for x in e[2][3:11]),e
command(3,8.,9.);allow[0]=True;run(100100,170000)
assert val('g_motor_zero_state')==5 and val('g_motor_zero_recovered')==1
allow[1]=True;run(170100,240000)
assert val('g_motor_zero_state')==6 and val('g_motor_zero_recovered')==3
command(3,10.,11.);run(240100,270000)
assert len(zeros())==2
assert all(e[2][3]==0 and e[2][8]==0 for e in events if e[0]=='start' and e[3]>=20100)
call('motor_zero_status_bit',now);assert u.reg_read(UC_ARM_REG_R0)==1
command(1,.7,-.2);run(270100,300000)
last=[next(e for e in reversed(events) if e[0]=='start' and e[1]==i) for i in range(2)]
assert abs(last[0][2][3]-.7)<1e-6 and abs(last[1][2][3]+.2)<1e-6 and all(e[2][8]==5 for e in last)
command(3,0.,0.);run(300100,400000);assert len(zeros())==4
# Premature release does not re-enable torque while a motor is still writing.
reset();command(3);run(0,30000);command(1,1.,-1.);run(30100,150000)
assert val('g_motor_zero_state')==5
assert all(e[2][3]==0 and e[2][8]==0 for e in events if e[0]=='start')
# Pending old cached mode1 feedback does not complete before first new probe TC.
reset();w(data[0]+100,100);w(data[1]+100,100);w(data[0]+48+8,1);w(data[1]+48+8,1)
command(3);run(0,70000);assert val('g_motor_zero_state')==5
# No debug mode => no zero request. Timing and hold work across TIM2 wrap.
reset();command(2);run(0,40000);assert not zeros()
reset(0xffffe000);command(3)
for d in range(0,60000,100):tick(0xffffe000+d)
assert len(zeros())==2
print('PASS actual linked ARM: left/right mode2 exactly once, >=10ms after leftTC, no mode2 ACK required, independent fresh mode1 recovery, stale cache rejected, switch2 holds torque/KP0 even after DONE, explicit OFF restores control, early OFF stays zero until recovery, no debug/no zero, rearm edge, timer wrap')
