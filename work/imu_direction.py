from pathlib import Path
import shutil,datetime,re
p=Path(r'C:/Users/libai/Desktop/X1/imu_caiji/zhuban_caiji/App');d=Path('tmp/dap_motor_diagnosis/imu_direction_'+datetime.datetime.now().strftime('%H%M%S'));d.mkdir()
for name in ['imu.h','imu_debug_mode.c']:
 f=p/name;shutil.copy2(f,d/(name+'.before'));s=f.read_text(encoding='utf-8-sig')
 if name=='imu.h':new=s.replace('/* 最终竖装映射（用户实测确认可用）：','/* 最终竖装映射（用户实测确认可用）：\n * 校正后的机身坐标：X轴指向人体前方，Y轴指向人体左侧（用户确认）。')
 else:new=s.replace('/* 最终竖装映射（用户实测确认）：XYZ=(原Z,原X,原Y)。','/* 最终竖装映射（用户实测确认）：XYZ=(原Z,原X,原Y)。\n     * 校正后的机身坐标：X轴指向人体前方，Y轴指向人体左侧（用户确认）。')
 assert new!=s
 strip=lambda x:re.sub(r'\s+','',re.sub(r'/\*.*?\*/|//[^\n]*','',x,flags=re.S))
 assert strip(s)==strip(new)
 f.write_text(new,encoding='utf-8-sig',newline='\r\n')
with Path('tmp/dap_motor_diagnosis/diagnosis.txt').open('a',encoding='utf-8') as f:f.write('\n2026-11-12 User-confirmed corrected IMU body axes documented in App/imu.h and App/imu_debug_mode.c: X points toward human front, Y toward human left. Comment-only; mapping/math/signs unchanged; non-comment tokens verified identical. No build/flash. Backups '+str(d)+'\n')
print('Updated and verified comment-only edits:',d)
