from pathlib import Path
exec(Path('tmp/dap_motor_diagnosis/control_mode1/test_zero.py').read_text(encoding='utf-8-sig'))
wf=lambda a,v:u.mem_write(a,struct.pack('<f',v))
def raw_frame(data1,data2,l=0.,rr=0.):
 p=struct.pack('<III8fI',0xaaaaaaaa,data1,data2,0.,0.,0.,0.,0.,l,rr,0.,0xeeeeeeee)
 h=r(sym('rx_head'))
 for x in p:u.mem_write(sym('rx_ring')+h,bytes([x]));h=(h+1)%1024
 w(sym('rx_head'),h)
def frame(flags,l=0.,rr=0.):raw_frame(1 if flags&2 else 0,1 if flags&1 else 0,l,rr)
def remote_poll(t):
 w(0x40000024,t);w(sym('tx_busy'),0);call('reserved_uart2_poll')
 return struct.unpack('<III8fI',bytes(u.mem_read(sym('tx_buffer'),48)))
reset();w(sym('g_uart2_tx_frame')+4,0xa0)
for i,ang,vel,tq in [(0,1.25,3.25,.1),(1,2.5,4.5,-.2)]:
 w(data[i]+112,1);wf(data[i]+48+20,ang);wf(data[i]+48+16,vel);wf(data[i]+48+12,tq)
frame(3,9.,-9.);a=remote_poll(50000)
assert a[1]==0xa1 and a[2]==1 and val('g_motor_zero_state')==1
assert a[4:8]==(1.25,2.5,3.25,4.5) and abs(a[8]-.1)<1e-6 and abs(a[9]+.2)<1e-6
assert remote_poll(300000)[1]==0xa0 and remote_poll(550000)[1]==0xa1
frame(3,8.,-8.);remote_poll(600000);assert val('g_motor_zero_state')==1
w(sym('g_motor_zero_state'),6);w(sym('g_motor_zero_recovered'),3)
assert remote_poll(700000)[1]==0xa1
frame(1,.7,-.2);a=remote_poll(750000)
assert a[1]==0xa0 and a[2]==1 and val('g_motor_zero_switch')==0
assert abs(struct.unpack('<f',u.mem_read(sym('g_motor_debug_torque_left'),4))[0]-.7)<1e-6
frame(0);a=remote_poll(800000);assert a[1]==0xa0 and a[2]==0 and val('g_motor_debug_mode')==0
for mode in [0,4,7,0x10003,0xffffffff]:
 reset();raw_frame(1,mode,.5,-.5);a=remote_poll(50000)
 assert a[2]==0 and val('g_motor_debug_mode')==0 and val('g_motor_zero_state')==0
reset();raw_frame(2,1,.5,-.5);a=remote_poll(50000)
assert a[2]==1 and val('g_motor_debug_mode')==1 and val('g_motor_zero_state')==0
raw_frame(1,1,1.,-1.);a=remote_poll(100000)
assert val('g_motor_zero_state')==1 and a[1]&1
print('PASS remapped full UART9 ARM integration: data2 motor mode exact equality1 only, legacy mode bits not accepted, data1bit0 zero edge and flash/steady/OFF ACK, data2 actual mode ACK, unrelated data1bits preserved, telemetry order and all zero-sequence regressions')

# New IMU-mode test: exact requested order, degree conversion, no yaw, force motor zero.
reset()
imu=sym('g_imu_data')
for off,v in [(0,math.pi/6),(4,-math.pi/4),(8,99.),(12,1.1),(16,-2.2),(20,3.3),(24,4.4),(28,-5.5),(32,9.81)]:wf(imu+off,v)
raw_frame(0,1,.75,-.5);remote_poll(50000);assert val('g_motor_debug_mode')==1
raw_frame(1,2,float('nan'),float('inf'));a=remote_poll(100000)
assert a[2]==2 and val('g_uart2_debug_mode')==2 and val('g_motor_debug_mode')==0
assert val('g_motor_zero_state')==0 and val('g_motor_zero_switch')==0
expected=[30.,-45.,1.1,-2.2,3.3,4.4,-5.5,9.81]
assert all(abs(x-y)<1e-4 for x,y in zip(a[3:11],expected)),a
assert abs(struct.unpack('<f',u.mem_read(sym('g_motor_debug_torque_left'),4))[0])==0
# Long quiet period does not leave IMU debug and does not start zero commands.
a=remote_poll(10100000);assert a[2]==2 and all(abs(x-y)<1e-4 for x,y in zip(a[3:11],expected))
# Frame builder after transition cannot retain old nonzero motor torque or KP5.
for obj,base in zip(objs,data):
 w(obj+592,0);call('start_motor_command_frame',obj,10100000,1)
 f=struct.unpack('<III7fII',bytes(u.mem_read(base,48)));assert f[2]==1 and all(x==0 for x in f[3:11])
# Motor mode resumes its existing fields and clears IMU acceleration leftover in data10.
raw_frame(0,1,.1,-.1);a=remote_poll(10150000);assert a[2]==1 and a[10]==0
# Unknown modes exit to ordinary telemetry with no IMU-mode stale fields.
raw_frame(0,4);a=remote_poll(10200000)
assert a[2]==0 and a[6:11]==(0.,0.,0.,0.,0.) and abs(a[4]-math.pi/6)<1e-6 and abs(a[5]+math.pi/4)<1e-6
print('PASS IMU mode actual ARM: mode2 ACK, Roll/Pitch degrees first then gyroXYZrad/s accelXYZm/s2, yaw excluded, motor torque/KP cleared on entry, quiet hold, mode switches and stale-field cleanup')
# Control mode1: exact whole uint32 value3, empty entry, no zero trigger, stationary cleanup.
for prior in [0,1,2]:
 reset();raw_frame(0,prior,.75,-.5);remote_poll(50000)
 raw_frame(1,3,float('nan'),float('inf'));a=remote_poll(100000)
 assert a[2]==3 and val('g_uart2_debug_mode')==3 and val('g_motor_debug_mode')==0
 assert val('g_motor_zero_state')==0 and val('g_motor_zero_switch')==0
 assert val('g_motor_debug_torque_left')==val('g_motor_debug_torque_right')==0
 for obj,base in zip(objs,data):
  w(obj+592,0);call('start_motor_command_frame',obj,100000,1)
  f=struct.unpack('<III7fII',bytes(u.mem_read(base,48)))
  assert f[2]==1 and all(x==0 for x in f[3:11])
 assert remote_poll(10100000)[2]==3
 for mode in [2,1,0,3,0x10003,7,0xffffffff]:
  raw_frame(0,mode,.1,-.1);a=remote_poll(10200000+mode%1000*50000)
  assert a[2]==(mode if mode in [1,2,3] else 0)
# Started zero sequence must safely finish after mode3 entry.
reset();command(3);run(0,1000)
assert val('g_motor_zero_state')!=0
raw_frame(0,3);remote_poll(1100)
allow[:]=[True,True];run(1200,120000)
assert len(zeros())==2 and val('g_motor_zero_state')==6
assert all(e[2][3]==0 and e[2][8]==0 for e in events if e[0]=='start')
print('PASS control mode1: exact data2=3 ACK, all prior modes transition, targets cleared, mode1 all-zero/KP0 builder, no new zero, quiet hold, exits and unknown values, active zero finishes safely')
