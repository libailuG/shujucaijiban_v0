from pathlib import Path
p=Path('work/gate_dir.txt');d=Path(p.read_text().strip())
base=Path('tmp/dap_motor_diagnosis/control_mode1/test_zero.py').read_text(encoding='utf-8-sig').split('objs=')[0]
base=base.replace("text=Path('tmp/dap_motor_diagnosis/control_mode1/symbols.txt')",f"text=Path(r'{d.as_posix()}/symbols.txt')")
base=base.replace('(0x08000000,0x20000)','(0x08000000,0x100000)').replace('(0x20000000,0x10000)','(0x20000000,0x20000),(0x24000000,0x80000)').replace('0x0801f001','0x080ff001').replace('0x0801f000','0x080ff000')
base+='''
def last(n):
 matches=re.findall(r'^\\s*\\d+\\s+'+re.escape(n)+r'\\s+(0x[0-9a-fA-F]+)',text,re.M);assert matches,n;return int(matches[-1],16)
def hook(uc,addr,size,unused):
 if addr==(sym('control_mode1_poll')&~1):uc.reg_write(UC_ARM_REG_PC,uc.reg_read(UC_ARM_REG_LR))
u.hook_add(UC_HOOK_CODE,hook)
u.mem_write(0x20000000,bytes(0x20000))
# Only policy compute is skipped; actual UART framing/acceptance and motor command path run.
for m in re.findall(r'^\\s*\\d+\\s+tx_busy\\s+(0x[0-9a-fA-F]+)',text,re.M):w(int(m,16),1)
def packet(mode,flags,tail=0xeeeeeeee):
 p=struct.pack('<III8fI',0xaaaaaaaa,flags,mode,*([0.]*8),tail);h=r(sym('rx_head'))
 for x in p:u.mem_write(sym('rx_ring')+h,bytes([x]));h=(h+1)%1024
 w(sym('rx_head'),h);call('reserved_uart2_poll')
def gate():return val('g_control_mode1_torque_enabled')
def ready():
 w(last('active'),1);w(sym('g_control_mode1_status'),0);w(sym('previous_tick'),1000);w(0x40000024,1000)
 u.mem_write(sym('command'),struct.pack('<2f',1.25,-2.5))
def get():
 call('control_mode1_get_command',0x2001e000)
 return u.reg_read(UC_ARM_REG_R0),struct.unpack('<2f',u.mem_read(0x2001e000,8))
def frames():
 out=[]
 for i,name in enumerate(['left','right']):
  obj=sym(name);data=sym('g_motor_'+name)
  for off,v in [(0,0x40004800 if i==0 else 0x40011400),(4,0x58020c00),(8,1),(12,126+i),(16,data),(20,0x24000000+i*64)]:w(obj+off,v)
  call('start_motor_command_frame',obj,1000,1)
  out.append(struct.unpack('<III7fII',u.mem_read(data,48)))
 return out
assert gate()==0
packet(3,0);ready();assert gate()==0 and get()==(0,(0.,0.))
assert all(f[2]==1 and all(v==0 for v in f[3:11]) for f in frames())
packet(3,1);ready();assert gate()==1 and get()==(1,(1.25,-2.5))
f=frames();assert f[0][3]==1.25 and f[1][3]==-2.5 and all(x[8]==10 for x in f)
# Event state survives silence, bad frames cannot close it.
packet(3,0,0);assert gate()==1
call('reserved_uart2_poll');assert gate()==1
packet(3,0);ready();assert gate()==0 and get()==(0,(0.,0.))
assert all(f[2]==1 and all(v==0 for v in f[3:11]) for f in frames())
packet(3,1);packet(2,1);assert gate()==0
packet(3,0);assert gate()==0
packet(3,9);assert gate()==0 and val('g_uart2_debug_mode')==0
packet(1,0);packet(1,1);assert gate()==0 and val('g_motor_zero_state')==1
assert val('g_motor_zero_left_sent')==0 and val('g_motor_zero_right_sent')==0
print('PASS actual ARM: startup default off, UART9 bit0 opens/closes mode3 torque, zero/KP0 when off, signed paired torque/KP10 when on, bad-frame rejection, event hold, exit/reentry, bit3 priority, mode1 zero edge preserved; no mode3 zero event')
'''
(d/'test_gate.py').write_text(base,encoding='utf-8')
