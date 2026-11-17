from pathlib import Path
import shutil,datetime
p=Path(r'C:/Users/libai/Desktop/X1/imu_caiji/zhuban_caiji');d=Path('tmp/dap_motor_diagnosis/control1_gate_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'));d.mkdir()
for rel in ['App/control_mode1.c','App/control_mode1.h','App/reserved_uart2.c','App/reserved_uart2.h','README.txt']:
 shutil.copy2(p/rel,d/(Path(rel).name+'.before'))
 def put(s): (p/rel).write_text(s,encoding='utf-8-sig',newline='\r\n')
 s=(p/rel).read_text(encoding='utf-8-sig')
 if rel=='App/control_mode1.c':
  s=s.replace('static float command[2];','''static float command[2];
/* 事件遥控开关：上电默认关闭，仅有效模式3命令的data1.bit0允许输出。
 * 关闭不停止策略计算/监测；电机层取不到命令时保持mode1全零/KP0。
 * 无消息超时退出；退出模式必须清除此开关，其他模式的bit0不授权策略输出。
 */
volatile uint32_t g_control_mode1_torque_enabled;
void control_mode1_set_torque_enabled(uint32_t enabled)
{
    g_control_mode1_torque_enabled = enabled ? 1U : 0U;
}
''')
  s=s.replace('if (g_uart2_debug_mode!=RESERVED_UART2_CONTROL_MODE1_VALUE || !active ||','if (!g_control_mode1_torque_enabled ||\n        g_uart2_debug_mode!=RESERVED_UART2_CONTROL_MODE1_VALUE || !active ||')
 elif rel=='App/control_mode1.h':
  s=s.replace('void control_mode1_reset(void);','''void control_mode1_reset(void);
/* 模式3 data1.bit0：默认0禁止策略扭矩下发，1允许；策略计算仍运行。 */
void control_mode1_set_torque_enabled(uint32_t enabled);
extern volatile uint32_t g_control_mode1_torque_enabled;''')
 elif rel=='App/reserved_uart2.c':
  s=s.replace('    g_uart2_debug_mode = 0U;','    g_uart2_debug_mode = 0U;\n    control_mode1_set_torque_enabled(0U);')
  anchor='requested_mode==RESERVED_UART2_CONTROL_MODE1_VALUE) ? requested_mode : 0U;'
  assert anchor in s
  s=s.replace(anchor,anchor+'''
                    /* 模式3的bit0仅控制策略扭矩输出；模式1的bit0仍为置零请求。
                     * bit3强制静止优先，按最终接受模式清开关，防止跨模式沿用授权。 */
                    control_mode1_set_torque_enabled(
                        g_uart2_debug_mode==RESERVED_UART2_CONTROL_MODE1_VALUE &&
                        (frame.data1 & RESERVED_UART2_CONTROL1_TORQUE_MASK));''')
 elif rel=='App/reserved_uart2.h':
  s=s.replace('#define RESERVED_UART2_ZERO_SWITCH_MASK', '#define RESERVED_UART2_CONTROL1_TORQUE_MASK (1UL << 0) /* 仅模式3：策略扭矩下发开关，默认关。 */\n#define RESERVED_UART2_ZERO_SWITCH_MASK')
 else:s='控制模式1扭矩门控（2026-11-13）：data2=3时，data1.bit0默认0禁止策略扭矩输出，1允许输出；关闭时电机仍为mode1全零/KP0，策略计算及监测继续。按最后有效遥控事件保持开关，不设消息超时；退出模式清开关，bit3强制普通静止优先。模式1的bit0仍是置零请求。\n\n'+s
 put(s)
Path('work/gate_dir.txt').write_text(str(d.resolve()));print(d)
