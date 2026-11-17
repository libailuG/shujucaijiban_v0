import sys,struct,re,math
from pathlib import Path
sys.path.insert(0,str(Path('tmp/protocol_test_deps').resolve()))
from unicorn import *
from unicorn.arm_const import *
elf=Path(r'C:/Users/libai/Desktop/X1/imu_caiji/zhuban_caiji/ObjectsCli/zhuban_led.axf').read_bytes()
text=Path('tmp/dap_motor_diagnosis/s2_control1/symbols_cli.txt').read_text(encoding='utf-8-sig')
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
w(sym('g_uart2_debug_mode'),3)
poll(100000)
assert value('g_control_mode1_selftest_error') < 0x38000000
for i in range(1,11):
 poll(100000+20000*i);assert value('g_control_mode1_status')==1;assert get()==(0.,0.)
poll(320000);assert value('g_control_mode1_status')==0
obs=floats(sym('output'),65)
assert abs(obs[54]-.2/math.pi)<1e-6 and abs(obs[55]-.3/math.pi)<1e-6,obs[54:60]
assert abs(obs[60])+abs(obs[61])<1e-6
out=floats(sym('output')+276,2);cmd=get()
assert abs(cmd[0]-out[0])<1e-6 and abs(cmd[1]+out[1])<1e-6,(cmd,out)
assert all(abs(x)<=6 for x in cmd)
# Missing feedback immediately clears cached command, including between inference slots.
w(sym('g_motor_right')+108,0);call('control_mode1_poll');assert get()==(0.,0.)
poll(340000);poll(360000);assert value('g_control_mode1_status')==1
# Exit/reset and re-entry must warm up from scratch.
w(sym('g_uart2_debug_mode'),0);call('control_mode1_poll');assert get()==(0.,0.)
w(sym('g_uart2_debug_mode'),3);poll(400000);poll(420000);assert value('g_control_mode1_status')==1
# Bad actual period and active zero sequence must clear output/state.
poll(450000);assert value('g_control_mode1_status')==0xffffffff;assert get()==(0.,0.)
w(sym('g_motor_zero_state'),5);poll(460000);assert get()==(0.,0.)
w(sym('g_motor_zero_state'),0);poll(480000);poll(500000);assert value('g_control_mode1_status')==1
# IMU and motor invalid fields are rejected by the mapped controller.
seed(520000);f(sym('g_imu_body_data'),float('nan'));call('control_mode1_poll');assert get()==(0.,0.)
print('PASS: network selftest, warmup, zero posture, joint signs, torque output signs/limit, stale feedback, exit/re-entry, period faults, zero inhibit, NaN')

# Nonzero body tilt/gyro and right hip velocity map into model axes.
call('control_mode1_reset');w(sym('g_uart2_debug_mode'),3)
poll(600000)
for i in range(1,12):
 t=600000+i*20000;seed(t)
 a=sym('g_imu_body_data');f(a,.12);f(a+4,-.2)
 f(a+12,1.);f(a+16,2.);f(a+20,3.)
 f(sym('g_motor_left')+48+16,.4);f(sym('g_motor_right')+48+16,-.5)
 call('control_mode1_poll')
obs=floats(sym('output'),65)
assert abs(obs[56]-.4/(4*math.pi))<1e-6 and abs(obs[57]-.5/(4*math.pi))<1e-6
assert all(abs(a-b)<1e-6 for a,b in zip(obs[62:65],[-2/(4*math.pi),1/(4*math.pi),3/(4*math.pi)]))
# q_A=R_y(pitch)*R_x(roll); transpose installation rotates gravity to B.
gax=math.sin(-.2);gay=-math.sin(.12)*math.cos(-.2);gaz=-math.cos(.12)*math.cos(-.2)
expected=[math.atan2(-gax,-gaz)/math.pi,math.atan2(-gay,-gaz)/math.pi]
assert all(abs(a-b)<1e-6 for a,b in zip(obs[60:62],expected)),(obs[60:62],expected)
cmd=get();assert any(abs(x)>1e-6 for x in cmd)
# Exercise real motor frame builder with algorithm command and current coefficient.
for i,n in enumerate(['left','right']):
 a=sym(n);data=sym('g_motor_left' if i==0 else 'g_motor_right')
 for off,v in [(0,0x40004800 if i==0 else 0x40011400),(4,0x58020c00 if i==0 else 0x58020800),(8,1<<10 if i==0 else 1<<8),(12,126+i),(16,data)]:w(a+off,v)
call('motor_bus_poll')
for i,n in enumerate(['g_motor_left','g_motor_right']):
 a=sym(n);assert r(a+8)==1
 assert abs(floats(a+12,1)[0]-cmd[i])<1e-6
 assert floats(a+32,1)[0]==10.
print('PASS: nonzero body tilt, gyro axis mapping, velocity signs, actual motor mode1 torque/KP frame builder')

# Full reserved UART2 mode3 telemetry carries driver commands, not measured torque.
w(sym('tx_busy'),0);call('reserved_uart2_poll')
frame=struct.unpack('<III8fI',u.mem_read(sym('tx_buffer'),48))
assert frame[2]==3 and all(abs(a-b)<1e-6 for a,b in zip(frame[8:10],cmd)),(frame,cmd)
assert abs(frame[4]-math.degrees(.2))<1e-4 and abs(frame[5]-math.degrees(-.3))<1e-4,frame
# Stale feedback zeros output telemetry on next 20Hz packet.
w(0x40000024,880000);w(sym('tx_busy'),0);call('reserved_uart2_poll')
frame=struct.unpack('<III8fI',u.mem_read(sym('tx_buffer'),48))
assert frame[2]==3 and frame[8:10]==(0.,0.)
assert frame[4:6]==(0.,0.)
print('PASS: full UART9 mode3 data8/9 output command telemetry and stale-feedback zero telemetry')
