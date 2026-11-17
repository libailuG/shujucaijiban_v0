from pathlib import Path
import shutil,datetime
p=Path(r'C:/Users/libai/Desktop/X1/imu_caiji/zhuban_caiji');a=p/'App'
d=Path(r'C:/Users/libai/Desktop/X1/dege_caiji/tmp/dap_motor_diagnosis/mode_modules_'+datetime.datetime.now().strftime('%H%M%S'));d.mkdir()
for rel in ['App/reserved_uart2.c','App/reserved_uart2.h','App/motor_bus.c','App/motor_bus.h','zhuban_led.uvprojx','build.ps1','README.txt','App/imu.c','App/imu.h']:
 shutil.copy2(p/rel,d/(Path(rel).name+'.before'))
shutil.copy2(p/'Objects/zhuban_led.axf',d/'before.axf')
shutil.copy2(p/'Objects/zhuban_led.hex',d/'before.hex')
def put(f,s):f.write_text(s,encoding='utf-8-sig',newline='\r\n')
s=(a/'reserved_uart2.c').read_text(encoding='utf-8-sig')
start=s.index('/* 控制模式1（UART9');end=s.index('void reserved_uart2_poll',start)
control=s[start:end];s=s[:start]+s[end:]
start=s.index('        if (g_uart2_debug_mode == RESERVED_UART2_IMU_MODE_VALUE) {',s.index('void reserved_uart2_poll'))
end=s.index('        td_mcu_frame_t frame =',start)
old=s[start:end]
imu=old.split(') {',1)[1].split('        } else if (g_motor_debug_mode) {')[0]
imu=imu.replace('            imu_get_body_data(&imu);','            ImuData imu;\n            imu_get_body_data(&imu);')
motor=old.split('        } else if (g_motor_debug_mode) {')[1].split('        } else {')[0]
normal=old.split('        } else {')[1].rsplit('        }',1)[0]
def telemetry(name,body,includes):
 for x in range(3,11):body=body.replace('g_uart2_tx_frame.data'+str(x),'frame->data'+str(x))
 return includes+'\nvoid '+name+'_fill_feedback(volatile td_mcu_frame_t *frame)\n{\n'+body+'}\n'
for name,description in [('normal_mode','普通模式：电压、原始坐标Roll/Pitch(rad)，其余反馈清零。'),('motor_debug_mode','电机调试：左右角度rad、速度rad/s、实际扭矩Nm；目标由data8/9给定。'),('imu_debug_mode','IMU调试：使用已确认的竖装机身坐标，角度为度，不含yaw。')]:
 extra='int motor_debug_set_command(uint32_t enabled, float left_torque, float right_torque);\n' if name=='motor_debug_mode' else ''
 put(a/(name+'.h'),f'#ifndef {name.upper()}_H\n#define {name.upper()}_H\n#include "mcu_uart_protocol.h"\n/* {description} */\nvoid {name}_fill_feedback(volatile td_mcu_frame_t *frame);\n'+extra+'#endif\n')
normal='    ImuData imu;\n    imu_get_data(&imu);\n    frame->data3 = g_battery_data.filtered_voltage_v;\n'+normal
put(a/'normal_mode.c',telemetry('normal_mode',normal,'#include "normal_mode.h"\n#include "imu.h"\n#include "battery.h"'))
motor='    frame->data3 = g_battery_data.filtered_voltage_v;\n'+motor
ms=(a/'motor_bus.c').read_text(encoding='utf-8-sig');st=ms.index('volatile uint32_t g_motor_debug_mode');en=ms.index('/* 置零开关=',st);setter=ms[st:en];ms=ms[:st]+ms[en:];ms=ms.replace('#include <math.h>','#include "motor_debug_mode.h"');put(a/'motor_bus.c',ms)
put(a/'motor_debug_mode.c',telemetry('motor_debug_mode',motor,'#include "motor_debug_mode.h"\n#include "motor_bus.h"\n#include "battery.h"\n#include <math.h>\n\n'+setter))
put(a/'imu_debug_mode.c',telemetry('imu_debug_mode',imu,'#include "imu_debug_mode.h"\n#include "imu.h"'))
put(a/'control_mode1.h','#ifndef CONTROL_MODE1_H\n#define CONTROL_MODE1_H\n/* data2=3时每轮主循环调用；在此模块编写控制模式1，禁止阻塞。 */\nvoid control_mode1_poll(void);\n#endif\n')
put(a/'control_mode1.c','#include "control_mode1.h"\n\n'+control)
s=s[:start]+'''        if (g_uart2_debug_mode == RESERVED_UART2_IMU_MODE_VALUE) {
            imu_debug_mode_fill_feedback(&g_uart2_tx_frame);
        } else if (g_motor_debug_mode) {
            motor_debug_mode_fill_feedback(&g_uart2_tx_frame);
        } else {
            /* 普通模式和空控制模式1暂共用普通反馈。 */
            normal_mode_fill_feedback(&g_uart2_tx_frame);
        }
'''+s[end:]
s=s.replace('#include "imu.h"\n#include "battery.h"','#include "normal_mode.h"\n#include "motor_debug_mode.h"\n#include "imu_debug_mode.h"\n#include "control_mode1.h"').replace('        ImuData imu;\n        imu_get_data(&imu);\n','').replace('        g_uart2_tx_frame.data3 = g_battery_data.filtered_voltage_v;\n','');put(a/'reserved_uart2.c',s)
f=a/'reserved_uart2.h';s=f.read_text(encoding='utf-8-sig').replace('/* 独立空入口：仅在控制模式1的主循环中调用，预留后续实现。 */\nvoid control_mode1_poll(void);','#include "control_mode1.h" /* 兼容原入口声明，实现在独立模块。 */');put(f,s)
f=a/'motor_bus.h';s=f.read_text(encoding='utf-8-sig').replace('#include "motor_protocol.h"','#include "motor_protocol.h"\n#include "motor_debug_mode.h"').replace('int motor_debug_set_command(uint32_t enabled, float left_torque, float right_torque);\n','');put(f,s)
names=['normal_mode','motor_debug_mode','imu_debug_mode','control_mode1']
f=p/'zhuban_led.uvprojx';b=f.read_bytes();anchor=b'              <FileName>reserved_uart2.c</FileName>';st=b.rfind(b'            <File>',0,b.index(anchor));entries=''
for name in names:
 for ext,typ in [('c',1),('h',5)]:entries+=f'            <File>\r\n              <FileName>{name}.{ext}</FileName>\r\n              <FileType>{typ}</FileType>\r\n              <FilePath>.\\App\\{name}.{ext}</FilePath>\r\n            </File>\r\n'
f.write_bytes(b[:st]+entries.encode()+b[st:])
f=p/'build.ps1';s=f.read_text(encoding='utf-8-sig').replace("'reserved_uart2','motor_bus'","'reserved_uart2','motor_bus','normal_mode','motor_debug_mode','imu_debug_mode','control_mode1'");s=s.replace('ObjectsCli/motor_bus.o\n','ObjectsCli/motor_bus.o '+' '.join('ObjectsCli/'+n+'.o' for n in names)+'\n');put(f,s)
f=p/'README.txt';s=f.read_text(encoding='utf-8-sig');put(f,'模式模块入口（本次只拆分代码，协议不变）：\nApp/normal_mode.c/.h：普通模式反馈。\nApp/motor_debug_mode.c/.h：电机调试目标接受、反馈；置零状态机/底层安全发送仍在motor_bus.c。\nApp/imu_debug_mode.c/.h：IMU调试上行，保留最终竖装映射。\nApp/control_mode1.c/.h：控制模式1，在control_mode1_poll函数内编写，目前为空。\nApp/reserved_uart2.c：UART9解析、实际模式接受/回执与模块分发。所有poll均不应阻塞。\n\n'+s)
Path('work/mode_modules_dir.txt').write_text(str(d),encoding='utf-8');print(d)
