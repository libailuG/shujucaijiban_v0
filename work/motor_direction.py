from pathlib import Path
import shutil,datetime,re
p=Path(r'C:/Users/libai/Desktop/X1/imu_caiji/zhuban_caiji/App');d=Path('tmp/dap_motor_diagnosis/motor_direction_'+datetime.datetime.now().strftime('%H%M%S'));d.mkdir()
comment='''/* 电机安装朝向（用户确认）：左侧电机正转时下落，右侧电机正转时上升。
 * 左侧=JP3/USART3/ID126，右侧=JP2/USART6/ID127。
 * 本接口保留电机原始反馈符号，不在读取时做方向取反。
 */
'''
for name in ['motor_bus.c','motor_bus.h','motor_debug_mode.c']:
 f=p/name;shutil.copy2(f,d/(name+'.before'));old=f.read_text(encoding='utf-8-sig')
 if name=='motor_bus.c':new=old.replace('int motor_get_feedback(unsigned is_right,MotorFrame *out)',comment+'int motor_get_feedback(unsigned is_right,MotorFrame *out)')
 elif name=='motor_bus.h':new=old.replace('int motor_get_feedback(unsigned right, MotorFrame *out);',comment+'int motor_get_feedback(unsigned right, MotorFrame *out);')
 else:new=old.replace('    motor_get_feedback(0U, &left); motor_get_feedback(1U, &right);','    /* 安装朝向：左电机正转下落，右电机正转上升；反馈保持各自原始符号。 */\n    motor_get_feedback(0U, &left); motor_get_feedback(1U, &right);')
 assert old!=new
 strip=lambda s:re.sub(r'\s+','',re.sub(r'/\*.*?\*/|//[^\n]*','',s,flags=re.S))
 assert strip(old)==strip(new),'Non-comment change'
 f.write_text(new,encoding='utf-8-sig',newline='\r\n')
with Path('tmp/dap_motor_diagnosis/diagnosis.txt').open('a',encoding='utf-8') as f:f.write('\n2026-11-12 User-confirmed motor mounting direction documented at motor_bus.c motor_get_feedback, motor_bus.h declaration, and motor_debug_mode.c feedback reads: LEFT positive rotation DOWN; RIGHT positive rotation UP. Original feedback signs preserved. Comment-only edits verified by comparing all non-comment tokens before/after; no build/flash/runtime change. Accurate backups: '+str(d)+'\n')
print('Comment-only changes verified; backup:',d)
