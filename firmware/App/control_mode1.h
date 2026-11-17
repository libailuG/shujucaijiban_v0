#ifndef CONTROL_MODE1_H
#define CONTROL_MODE1_H
#include <stdint.h>
#include "mcu_uart_protocol.h"
#include "control_mode1_monitor.h"
/* data2=1；只在主循环调用，算法坐标与单位适配均封装在本模块。 */
void control_mode1_poll(void);
void control_mode1_reset(void);
/* 模式1 data1.bit0：默认0禁止策略扭矩下发，1允许；策略计算仍运行。 */
void control_mode1_set_torque_enabled(uint32_t enabled);
extern volatile uint32_t g_control_mode1_torque_enabled;
/* 模式1 data1.bit1：预留串口1上传SI单位的最新11项网络观测，默认关闭。 */
void control_mode1_set_observation_enabled(uint32_t enabled);
extern volatile uint32_t g_control_mode1_observation_enabled;
/* data4/5：左右电机角度[度]，保留驱动符号；data8/9：输出命令Nm，非实测扭矩。 */
void control_mode1_fill_feedback(volatile td_mcu_frame_t *frame);
int control_mode1_get_command(float driver_nm[2]);
extern volatile int32_t g_control_mode1_status;
extern volatile uint32_t g_control_mode1_ticks, g_control_mode1_last_compute_us;
extern volatile float g_control_mode1_selftest_error;
#endif

