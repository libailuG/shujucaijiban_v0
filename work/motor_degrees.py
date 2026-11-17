from pathlib import Path
import shutil,datetime
p=Path(r'C:/Users/libai/Desktop/X1/imu_caiji/zhuban_caiji');d=Path('tmp/dap_motor_diagnosis/motor_angle_deg_'+datetime.datetime.now().strftime('%H%M%S'));d.mkdir()
for rel in ['App/motor_debug_mode.c','App/motor_debug_mode.h','App/reserved_uart2.c','README.txt']:
 shutil.copy2(p/rel,d/(Path(rel).name+'.before'))
 f=p/rel;s=f.read_text(encoding='utf-8-sig')
 if rel.endswith('motor_debug_mode.c'):
  s=s.replace('    frame->data4 = left.ang; frame->data5 = right.ang;','    /* 电机原始反馈为rad，仅在调试上行转换为度；角速度仍为rad/s。 */\n    const float rad_to_deg = 57.29577951308232f;\n    frame->data4 = left.ang * rad_to_deg;\n    frame->data5 = right.ang * rad_to_deg;')
 s=s.replace('左右角度rad','左右角度[度]').replace('左/右角度rad','左/右角度[度]')
 f.write_text(s,encoding='utf-8-sig',newline='\r\n')
old=Path(Path('work/mode_modules_dir.txt').read_text().strip());rel=d.as_posix()
for name in ['test_zero.py','test_remote.py']:
 s=(old/name).read_text(encoding='utf-8-sig').replace(old.relative_to(Path.cwd()).as_posix(),rel)
 if name=='test_remote.py':
  s=s.replace('assert a[4:8]==(1.25,2.5,3.25,4.5)', 'assert abs(a[4]-math.degrees(1.25))<1e-4 and abs(a[5]-math.degrees(2.5))<1e-4 and a[6:8]==(3.25,4.5)')
  s+='''\n# Positive and negative radian feedback must convert to degrees only on motor telemetry.\nreset()\nfor i,angle in [(0,math.pi/2),(1,-math.pi)]:\n w(data[i]+112,1);wf(data[i]+48+20,angle)\nraw_frame(0,1);a=remote_poll(50000)\nassert abs(a[4]-90)<1e-4 and abs(a[5]+180)<1e-4,a\nprint('PASS motor data4/5 degrees: left +pi/2 -> +90, right -pi -> -180; speed rad/s and torque Nm preserved')\n'''
 (d/name).write_text(s,encoding='utf-8')
Path('work/degrees_dir.txt').write_text(str(d.resolve()));print(d)
