from pathlib import Path
p=Path(r'C:/Users/libai/Desktop/X1/imu_caiji/zhuban_caiji/App')
for name in ['normal_mode','motor_debug_mode','imu_debug_mode']:
 f=p/(name+'.c');s=f.read_text(encoding='utf-8-sig');s='\n'.join(line[8:] if line.startswith('            ') else line for line in s.split('\n'));f.write_text(s,encoding='utf-8-sig',newline='\r\n')
f=p/'motor_bus.h';s=f.read_text(encoding='utf-8-sig');start=s.index('/* Event-based remote:');end=s.index('/* 状态：',start);decl=s[start:end];s=s[:start]+s[end:];f.write_text(s,encoding='utf-8-sig',newline='\r\n');f=p/'motor_debug_mode.h';s=f.read_text(encoding='utf-8-sig').replace('void motor_debug_mode_fill_feedback',decl+'void motor_debug_mode_fill_feedback');f.write_text(s,encoding='utf-8-sig',newline='\r\n')
d=Path(Path('work/mode_modules_dir.txt').read_text().strip());rel=d.relative_to(Path.cwd()).as_posix()
for name in ['test_zero.py','test_remote.py']:
 s=(Path('tmp/dap_motor_diagnosis/control_mode1')/name).read_text(encoding='utf-8-sig').replace('tmp/dap_motor_diagnosis/control_mode1',rel)
 if name=='test_remote.py':
  s=s.replace("wf(imu+off,v)","wf(imu+off,v);wf(sym('g_imu_body_data')+off,v)")
 (d/name).write_text(s,encoding='utf-8')
