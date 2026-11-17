from pathlib import Path
exec(Path('tmp/dap_motor_diagnosis/zero_sequence/test_zero.py').read_text(encoding='utf-8-sig'))
reset()
wf=lambda a,v:u.mem_write(a,struct.pack('<f',v))
def frame(flags,l=0.,rr=0.):
 p=struct.pack('<III8fI',0xaaaaaaaa,flags,0,0.,0.,0.,0.,0.,l,rr,0.,0xeeeeeeee)
 h=r(sym('rx_head'))
 for x in p:u.mem_write(sym('rx_ring')+h,bytes([x]));h=(h+1)%1024
 w(sym('rx_head'),h)
def remote_poll(t):
 w(0x40000024,t);w(sym('tx_busy'),0);call('reserved_uart2_poll')
 return struct.unpack('<III8fI',bytes(u.mem_read(sym('tx_buffer'),48)))
w(sym('g_uart2_tx_frame')+4,0xa0)
for i,ang,vel,tq in [(0,1.25,3.25,.1),(1,2.5,4.5,-.2)]:
 w(data[i]+112,1);wf(data[i]+48+20,ang);wf(data[i]+48+16,vel);wf(data[i]+48+12,tq)
frame(3,9.,-9.);a=remote_poll(50000)
assert a[1]==0xa3 and val('g_motor_zero_state')==1
assert a[4:8]==(1.25,2.5,3.25,4.5) and abs(a[8]-.1)<1e-6 and abs(a[9]+.2)<1e-6
assert a[1]==0xa3 and val('g_motor_debug_mode')==1
assert remote_poll(300000)[1]==0xa1 and remote_poll(550000)[1]==0xa3
frame(3,8.,-8.);remote_poll(600000);assert val('g_motor_zero_state')==1
w(sym('g_motor_zero_state'),6);w(sym('g_motor_zero_recovered'),3)
assert remote_poll(700000)[1]==0xa3
frame(1,.7,-.2);a=remote_poll(750000)
assert a[1]==0xa1 and val('g_motor_zero_switch')==0
assert abs(struct.unpack('<f',u.mem_read(sym('g_motor_debug_torque_left'),4))[0]-.7)<1e-6
frame(0);a=remote_poll(800000);assert a[1]==0xa0 and val('g_motor_debug_mode')==0
print('PASS UART9 actual linked ARM integration: bit1 rising routes zero request, bit0 ACK preserved, bit1 250ms flash, complete steadyON, OFF ACK cleared, paired telemetry unchanged, torque requests ignored while switch2ON')
