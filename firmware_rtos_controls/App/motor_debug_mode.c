#include "motor_debug_mode.h"
#include "motor_bus.h"
#include "battery.h"
#include <math.h>

volatile uint32_t g_motor_debug_mode, g_motor_debug_reject_count;
volatile float g_motor_debug_torque_left, g_motor_debug_torque_right;
/* 遥控器按事件发送，并非周期心跳（用户确认2026-11-12）：
 * 接收data2数值恰好等于11后保持最后一条有效扭矩，不按静默时间自动退出。
 * data2不等于11时恢复mode1零扭矩、KP0，禁止使用mode0。
 * 打开命令中任意扭矩为NaN/Inf时拒绝整条命令，保留已接受状态；
 * 关闭命令不依赖扭矩字段合法性，始终允许退出。仅主循环调用。 */
int motor_debug_set_command(uint32_t enabled, float left_torque, float right_torque)
{
    if (enabled && (!isfinite(left_torque) || !isfinite(right_torque))) {
        ++g_motor_debug_reject_count;
        return 0;
    }
    g_motor_debug_torque_left = enabled ? left_torque : 0.0f;
    g_motor_debug_torque_right = enabled ? right_torque : 0.0f;
    g_motor_debug_mode = enabled ? 1U : 0U;
    return 1;
}

void motor_debug_mode_fill_feedback(volatile td_mcu_frame_t *frame)
{
    frame->data3 = g_battery_data.filtered_voltage_v;

    MotorFrame left = {0}, right = {0};
    /* 安装朝向：左电机正转下落，右电机正转上升；反馈保持各自原始符号。 */
    motor_get_feedback(0U, &left); motor_get_feedback(1U, &right);
    frame->data10 = right.torque; /* 右电机反馈扭矩，Nm，保留原始符号。 */
    /* 电机原始反馈为rad，仅在调试上行转换为度；角速度仍为rad/s。 */
    const float rad_to_deg = 57.29577951308232f;
    frame->data4 = left.ang * rad_to_deg;
    frame->data5 = right.ang * rad_to_deg;
    frame->data6 = left.speed; frame->data7 = right.speed;
    frame->data8 = left.torque; frame->data9 = right.torque;
}

