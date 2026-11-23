#include "normal_mode.h"
#include "imu.h"
#include "battery.h"
#include "motor_bus.h"
void normal_mode_fill_feedback(volatile td_mcu_frame_t *frame)
{
    ImuData imu;
    imu_get_body_data(&imu);
    const float rad_to_deg = 57.29577951308232f;
    frame->data3 = g_battery_data.filtered_voltage_v;

    frame->data10 = 0.0f;
    /* 普通模式：使用最终安装校正后的机身姿态，单位度。 */
    frame->data4 = imu.angle_rad.roll * rad_to_deg;
    frame->data5 = imu.angle_rad.pitch * rad_to_deg;
    MotorFrame left = {0}, right = {0};
    motor_get_feedback(0U, &left);
    motor_get_feedback(1U, &right);
    /* 普通模式：data6/7为左右电机反馈角度[度]，保持原始符号。 */
    frame->data6 = left.ang * rad_to_deg;
    frame->data7 = right.ang * rad_to_deg;
    frame->data8 = frame->data9 = 0.0f;
}
