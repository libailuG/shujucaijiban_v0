from pathlib import Path
exec(Path('tmp/dap_motor_diagnosis/switch_remap/test_zero.py').read_text(encoding='utf-8-sig'))
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
for mode in [0,2,3,0xffffffff]:
 reset();raw_frame(1,mode,.5,-.5);a=remote_poll(50000)
 assert a[2]==0 and val('g_motor_debug_mode')==0 and val('g_motor_zero_state')==0
reset();raw_frame(2,1,.5,-.5);a=remote_poll(50000)
assert a[2]==1 and val('g_motor_debug_mode')==1 and val('g_motor_zero_state')==0
raw_frame(1,1,1.,-1.);a=remote_poll(100000)
assert val('g_motor_zero_state')==1 and a[1]&1
print('PASS remapped full UART9 ARM integration: data2 exact equality1 only, legacy mode bits not accepted, data1bit0 zero edge and flash/steady/OFF ACK, data2 actual mode ACK, unrelated data1bits preserved, telemetry order and all zero-sequence regressions')
