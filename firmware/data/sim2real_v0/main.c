/*
 * DS v17 接入示例：只需 main.c + s2_deploy.c + s2_deploy.h。
 * 下列 board_* 是占位函数，替换成你自己的 HAL/RTOS/驱动实现。
 * 当前默认只做网络自检后退出，不连接电机，不伪造有效传感器数据。
 * 已有 STM32 main.c 时，将初始化/20ms任务中的逻辑拷贝到已有工程，
 * 不要同时链接两个 main()。详细坐标、方向、单位和注意事项见 s2_deploy.h。
 */
#include "s2_deploy.h"
#include <string.h>

/* 替换为时钟、FPU、IMU、编码器和电机通信初始化。 */
static void board_init(void) { }

/* 替换为20ms定时事件/RTOS周期等待；1=到期，0=示例结束。
 * 不要简单用忙循环代替固定50Hz任务。 */
static int board_wait_20ms(void) { return 0; }

/* 只在同步快照新鲜、有效时返回1；失败时返回0。
 * 填实际时间间隔measured_dt_s、quat_ws、gyro_sensor_rad_s、
 * 输出关节端左右编码器、使能和sensors_valid；不默认假设有效。 */
static int board_read_sensors(S2HardwareInput *sample) {
    memset(sample, 0, sizeof(*sample));
    return 0;
}

/* 必须每周期发送，包括0扭矩；返回1表示成功。
 * 此处单位为驱动符号下的输出关节端Nm，不是A、电机转子Nm或协议整数。
 * 替换成实际驱动命令；扭矩到电流/齿比/协议缩放在这里做。 */
static int board_send_joint_torque(const float driver_nm[2]) {
    (void)driver_nm;
    return 0; /* 未接驱动，不伪报成功。 */
}

/* 替换为故障锁存、驱动禁止输出及人工重新使能流程。
 * 通信发送失败时仅发送零还不够，需要驱动看门狗/禁用链路。 */
static void board_disable_drive(void) { }

static void configure_calibration(void) {
    /* 下列均为待标定占位值。方向定义见头文件，不能直接照抄到实机。 */
    s2_board_config.quat_sb[0] = 1.0f; /* IMU +X右 +Y前 +Z上时的安装旋转。 */
    s2_board_config.quat_sb[1] = 0.0f;
    s2_board_config.quat_sb[2] = 0.0f;
    s2_board_config.quat_sb[3] = 0.0f;
    s2_board_config.encoder_sign[0] = 1.0f;
    s2_board_config.encoder_sign[1] = 1.0f;
    s2_board_config.encoder_zero_rad[0] = 0.0f;
    s2_board_config.encoder_zero_rad[1] = 0.0f;
    s2_board_config.motor_sign[0] = 1.0f;
    s2_board_config.motor_sign[1] = 1.0f;
    s2_board_config.calibration_confirmed = 0; /* 完成实测后才能设1。 */
    s2_app_config.gyro_mode = S2_GYRO_SENSOR;
    s2_app_config.torque_limit_nm = 2.0f; /* 示例初始限幅，不是原版20Nm复现。 */
    s2_app_config.slew_rate_nm_s = 0.0f;
}

int main(void) {
    /* 静态分配，避免把约1.8KB自检工作区放在小栈上。 */
    static S2PolicyWorkspace selftest_workspace;
    S2HardwareInput sample;
    float driver_nm[2] = {0, 0}, selftest_error;
    S2Status status;
    board_init();
    board_disable_drive();
    s2_app_init();
    configure_calibration();
    if (s2_selftest(&selftest_workspace, &selftest_error) != 0) {
        board_disable_drive();
        return 1; /* 实机可替换为保持禁用的故障处理。 */
    }
    while (board_wait_20ms()) {
        if (!board_read_sensors(&sample)) {
            memset(&sample, 0, sizeof(sample));
        }
        /* 错误/禁用/暖机时，本函数同样给出明确的零命令。 */
        status = s2_app_tick_50hz(&sample, driver_nm);
        if (status < 0) {
            board_disable_drive();
        }
        if (!board_send_joint_torque(driver_nm)) {
            /* 清空策略历史；先禁用，再交给硬件层锁存/人工重新使能。 */
            board_disable_drive();
            memset(&sample, 0, sizeof(sample));
            (void)s2_app_tick_50hz(&sample, driver_nm);
        }
    }
    board_disable_drive();
    return 0;
}
