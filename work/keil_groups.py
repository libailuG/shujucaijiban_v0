from pathlib import Path
import re,shutil,datetime,xml.etree.ElementTree as ET
p=Path(r'C:/Users/libai/Desktop/X1/imu_caiji/zhuban_caiji/zhuban_led.uvprojx')
d=Path('tmp/dap_motor_diagnosis/keil_groups_'+datetime.datetime.now().strftime('%H%M%S'));d.mkdir()
shutil.copy2(p,d/p.name)
b=p.read_bytes();m=re.search(rb'      <Groups>.*?      </Groups>',b,re.S);assert m
blocks=re.findall(rb'            <File>.*?            </File>\r?\n',m[0],re.S)
files={re.search(rb'<FileName>(.*?)</FileName>',x)[1].decode():x for x in blocks}
groups=[('Core',['main.c']),('Startup',['system_stm32h7xx.c','startup_stm32h723xx.s']),('Sensors',['imu.c','imu_bus.c','icm_imu_i2c.c','mahony.c','battery.c']),('Communication',['reserved_uart2.c','motor_bus.c']),('Mode_Normal',['normal_mode.c','normal_mode.h']),('Mode_Motor_Debug',['motor_debug_mode.c','motor_debug_mode.h']),('Mode_IMU_Debug',['imu_debug_mode.c','imu_debug_mode.h']),('Mode_Control1',['control_mode1.c','control_mode1.h'])]
assert sorted(files)==sorted(x for _,names in groups for x in names)
new=b'      <Groups>\r\n'
for name,names in groups:
 new+=f'        <Group>\r\n          <GroupName>{name}</GroupName>\r\n          <Files>\r\n'.encode()+b''.join(files[x] for x in names)+b'          </Files>\r\n        </Group>\r\n'
new+=b'      </Groups>';out=b[:m.start()]+new+b[m.end():];ET.fromstring(out)
oldroot=ET.fromstring(b);newroot=ET.fromstring(out)
assert sorted(re.findall(rb'            <File>.*?            </File>',b,re.S))==sorted(re.findall(rb'            <File>.*?            </File>',out,re.S))
p.write_bytes(out)
Path('work/groups_dir.txt').write_text(str(d.resolve()))
print('8 groups, all 18 file entries preserved exactly; non-group project bytes unchanged. Backup:',d)

