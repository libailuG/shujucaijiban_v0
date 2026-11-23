#include "control_mode1.h"
#include "control1_policy/s2_deploy.h"
#include "imu.h"
#include "imu_bus.h"
#include "motor_bus.h"
#include "reserved_uart2.h"
#include "reserved_uart1.h"
#include "normal_mode.h"
#include <math.h>
#include <string.h>

/* 算法的方向/单位适配仅在本模块：穿戴者左0/右1。
 * 电机反馈及命令都是关节端rad、rad/s、Nm，无齿比或单位缩放。
 * 编码器0为机械零点；编码器与扭矩同向。左正转向后、右正转向前，
 * 模型两侧以向后伸展为正，故encoder_sign/motor_sign均为[+1,-1]。
 * 输入机身A为X前Y左Z上，模型B为X右Y前Z上。使用quat_sb旋转，
 * gyro_B=[-gyro_A.y,gyro_A.x,gyro_A.z]，不直接交换欧拉角。
 * 2026-11-13安装方向纠正在imu.c机身映射层完成，本层不重复反向。
 * 模式1运行50Hz策略，电机层100Hz发送保持值；普通上行协议不变。
 * 使用S2 v30默认12Nm限幅，10周期零扭矩暖机；未添加仿真中的被动阻尼。
 * 遥控器为事件协议，无静默退出；模式退出及置零期间清空状态。
 */
#define PERIOD_US 20000U
#define MOTOR_MAX_AGE_US 30000U
volatile ControlMode1Monitor g_control_mode1_monitor = {
    .start_flag = CONTROL_MODE1_MONITOR_START_FLAG,
    .version = CONTROL_MODE1_MONITOR_VERSION,
    .frame_size = sizeof(ControlMode1Monitor),
    .status = S2_DISABLED,
    .end_flag = CONTROL_MODE1_MONITOR_END_FLAG
};

static S2Controller controller;
static S2Config config;
static S2PortConfig port;
static S2Output output;
static uint32_t initialized, selftest_ok, active, previous_tick;
static float command[2];
/* Diagnostic bypass: feedback freshness/errors do not stop policy execution. */
static ImuData retained_imu;
static MotorFrame retained_motor[2];
/* 事件遥控开关：上电默认关闭，仅有效模式1命令的data1.bit0允许输出。
 * 关闭不停止策略计算/监测；电机层取不到命令时保持mode1全零/KP0。
 * 无消息超时退出；退出模式必须清除此开关，其他模式的bit0不授权策略输出。
 */
volatile uint32_t g_control_mode1_torque_enabled;
volatile uint32_t g_control_mode1_observation_enabled;
void control_mode1_set_observation_enabled(uint32_t enabled)
{
    g_control_mode1_observation_enabled=enabled ? 1U : 0U;
}
void control_mode1_set_torque_enabled(uint32_t enabled)
{
    g_control_mode1_torque_enabled = enabled ? 1U : 0U;
}

volatile int32_t g_control_mode1_status;
volatile uint32_t g_control_mode1_ticks, g_control_mode1_last_compute_us;
volatile float g_control_mode1_selftest_error;

void control_mode1_reset(void)
{
    s2_controller_reset(&controller);
    command[0] = command[1] = 0.0f;
    active = 0U;
    g_control_mode1_status = S2_DISABLED;
}

static int read_input(uint32_t now, S2HardwareInput *hw)
{
    (void)now;
    ImuData candidate;
    imu_get_body_data(&candidate);
    if (isfinite(candidate.angle_rad.roll) && isfinite(candidate.angle_rad.pitch) &&
        isfinite(candidate.angle_rad.yaw) && isfinite(candidate.gyro_rad_s.x) &&
        isfinite(candidate.gyro_rad_s.y) && isfinite(candidate.gyro_rad_s.z))
        retained_imu=candidate;
    for (unsigned side=0;side<2;++side) {
        MotorFrame f;
        if (motor_get_feedback(side,&f) && isfinite(f.ang) && isfinite(f.speed))
            retained_motor[side]=f;
    }
    ImuData imu=retained_imu;
    MotorFrame left=retained_motor[0], right=retained_motor[1];
    /* 主板输出ZYX欧拉角(rad)，重建Hamilton机身->世界四元数。 */
    float cr=cosf(imu.angle_rad.roll*0.5f), sr=sinf(imu.angle_rad.roll*0.5f);
    float cp=cosf(imu.angle_rad.pitch*0.5f), sp=sinf(imu.angle_rad.pitch*0.5f);
    float cy=cosf(imu.angle_rad.yaw*0.5f), sy=sinf(imu.angle_rad.yaw*0.5f);
    memset(hw,0,sizeof(*hw));
    hw->quat_ws[0]=cr*cp*cy+sr*sp*sy;
    hw->quat_ws[1]=sr*cp*cy-cr*sp*sy;
    hw->quat_ws[2]=cr*sp*cy+sr*cp*sy;
    hw->quat_ws[3]=cr*cp*sy-sr*sp*cy;
    hw->gyro_sensor_rad_s[0]=imu.gyro_rad_s.x;
    hw->gyro_sensor_rad_s[1]=imu.gyro_rad_s.y;
    hw->gyro_sensor_rad_s[2]=imu.gyro_rad_s.z;
    hw->encoder_joint_rad[0]=left.ang; hw->encoder_joint_rad[1]=right.ang;
    hw->encoder_joint_rad_s[0]=left.speed; hw->encoder_joint_rad_s[1]=right.speed;
    hw->enabled=hw->sensors_valid=1U;
    return 1;
}

/* Build an independent snapshot before IRQ-driven UART transmission. */
static uint32_t monitor_sequence, monitor_last_us, monitor_started;
static void monitor_publish(uint32_t now, const S2HardwareInput *hw,
                            S2Status status, uint32_t compute_us, int stepped)
{
    if (!stepped && monitor_started && (uint32_t)(now-monitor_last_us)<PERIOD_US) return;
    monitor_started=1U;
    monitor_last_us=now;
    ControlMode1Monitor m={0};
    m.start_flag=CONTROL_MODE1_MONITOR_START_FLAG;
    m.end_flag=CONTROL_MODE1_MONITOR_END_FLAG;
    m.version=CONTROL_MODE1_MONITOR_VERSION;
    m.frame_size=sizeof(m);
    m.sequence=++monitor_sequence;
    m.timestamp_us=now;
    m.status=status;
    m.compute_time_us=compute_us;
    if (g_motor_left.rx_valid && (uint32_t)(now-g_motor_left.rx_timestamp_us)<=MOTOR_MAX_AGE_US)
        m.valid_flags|=CONTROL_MODE1_MONITOR_LEFT_VALID;
    if (g_motor_right.rx_valid && (uint32_t)(now-g_motor_right.rx_timestamp_us)<=MOTOR_MAX_AGE_US)
        m.valid_flags|=CONTROL_MODE1_MONITOR_RIGHT_VALID;
    if (g_imu_body_data.valid && g_imu_body_data.angle_valid && !g_imu_body_data.last_error &&
        (uint32_t)(now-g_imu_body_data.timestamp_us)<=IMU_SAMPLE_MAX_AGE_US)
        m.valid_flags|=CONTROL_MODE1_MONITOR_IMU_VALID;
    if (hw) {
        m.dt_s=hw->measured_dt_s;
        memcpy(m.joint_angle_rad,hw->encoder_joint_rad,sizeof(m.joint_angle_rad));
        memcpy(m.joint_speed_rad_s,hw->encoder_joint_rad_s,sizeof(m.joint_speed_rad_s));
        memcpy(m.body_quat,hw->quat_ws,sizeof(m.body_quat));
        memcpy(m.body_gyro_rad_s,hw->gyro_sensor_rad_s,sizeof(m.body_gyro_rad_s));
    }
    if (stepped && (status==S2_OK || status==S2_WARMUP)) {
        m.valid_flags|=CONTROL_MODE1_MONITOR_OBS_VALID;
        memcpy(m.joint_angle_norm,&output.obs[54],sizeof(m.joint_angle_norm));
        memcpy(m.joint_speed_norm,&output.obs[56],sizeof(m.joint_speed_norm));
        memcpy(m.previous_torque_norm,&output.obs[58],sizeof(m.previous_torque_norm));
        memcpy(m.body_tilt_norm,&output.obs[60],sizeof(m.body_tilt_norm));
        memcpy(m.body_gyro_norm,&output.obs[62],sizeof(m.body_gyro_norm));
        if (status==S2_OK) {
            m.valid_flags|=CONTROL_MODE1_MONITOR_NETWORK_VALID;
            memcpy(m.raw_action,output.raw,sizeof(m.raw_action));
            memcpy(m.requested_torque_nm,output.requested_nm,sizeof(m.requested_torque_nm));
            memcpy(m.model_torque_nm,output.command_nm,sizeof(m.model_torque_nm));
            if (control_mode1_get_command(m.driver_torque_nm))
                m.valid_flags|=CONTROL_MODE1_MONITOR_COMMAND_VALID;
        }
    }
    g_control_mode1_monitor=m;
    /* Diagnostic bypass may publish retained input; 52-byte frame has no
     * freshness flag. Debugger monitor retains actual freshness diagnostics. */
    if (stepped && (status==S2_OK || status==S2_WARMUP) &&
        g_control_mode1_observation_enabled) {
        ControlMode1Observation observation;
        control_mode1_observation_from_obs(&observation,output.obs);
        (void)reserved_uart1_send_observation(&observation);
    }
}

void control_mode1_poll(void)
{
    if (g_uart2_debug_mode!=RESERVED_UART2_CONTROL_MODE1_VALUE) {
        control_mode1_set_observation_enabled(0U);
        monitor_started=0U;
        control_mode1_reset(); return;
    }
    if (!initialized) {
        s2_config_default(&config);
        config.gyro_mode=S2_GYRO_SENSOR;
        s2_port_config_default(&port);
        port.quat_sb[0]=0.7071067812f; port.quat_sb[3]=-0.7071067812f;
        port.encoder_sign[0]=port.motor_sign[0]=1.0f;
        port.encoder_sign[1]=port.motor_sign[1]=-1.0f;
        port.encoder_zero_rad[0]=port.encoder_zero_rad[1]=0.0f;
        port.calibration_confirmed=1U; /* 用户已确认方向、零点、关节端单位。 */
        float error;
        selftest_ok=s2_selftest(&controller.work,&error)==0;
        g_control_mode1_selftest_error=error;
        initialized=1U;
    }
    uint32_t now=imu_time_us();
    S2HardwareInput hw;
    if (!selftest_ok || g_motor_zero_switch ||
        (g_motor_zero_state!=MOTOR_ZERO_IDLE && g_motor_zero_state!=MOTOR_ZERO_DONE) ||
        !read_input(now,&hw)) {
        control_mode1_reset();
        g_control_mode1_status=selftest_ok ? S2_BAD_INPUT : S2_NUMERIC_ERROR;
        monitor_publish(now,0,(S2Status)g_control_mode1_status,0U,0);
        return;
    }
    if (!active) { active=1U; previous_tick=now; return; }
    uint32_t elapsed=now-previous_tick;
    if (elapsed<PERIOD_US) return;
    previous_tick=now; /* 不补跑积压周期，实际周期异常由策略拒绝并清空历史。 */
    hw.measured_dt_s=(float)elapsed*1.0e-6f;
    S2Input model;
    command[0]=command[1]=0.0f;
    S2Status status=S2_BAD_INPUT;
    if (!s2_port_map_input(&port,&hw,&model)) {
        status=s2_controller_step(&controller,&config,&model,&output);
        if (status==S2_OK && s2_port_map_output(&port,output.command_nm,command))
            status=S2_BAD_INPUT;
    }
    if (status<0) { control_mode1_reset(); }
    g_control_mode1_status=status;
    ++g_control_mode1_ticks;
    g_control_mode1_last_compute_us=imu_time_us()-now;
    monitor_publish(now,&hw,status,g_control_mode1_last_compute_us,1);
}

/* 电机层只取驱动坐标下的命令，不包含算法方向/单位知识。失败保持零帧。 */
int control_mode1_get_command(float driver_nm[2])
{
    driver_nm[0]=driver_nm[1]=0.0f;
    if (!g_control_mode1_torque_enabled ||
        g_uart2_debug_mode!=RESERVED_UART2_CONTROL_MODE1_VALUE || !active ||
        g_control_mode1_status!=S2_OK || (uint32_t)(imu_time_us()-previous_tick)>21000U)
        return 0;
    driver_nm[0]=command[0]; driver_nm[1]=command[1];
    return 1;
}

/* 预留串口2(JP7/UART9)上行：data4/5为电机角度[度]，data8/9为输出命令。
 * 与电机发帧使用相同命令接口；暖机/异常/退出时反馈零，单位关节端Nm。
 * 这是当前输出设定值，不是电机实测扭矩，也不是发送完成ACK。
 */
void control_mode1_fill_feedback(volatile td_mcu_frame_t *frame)
{
    float driver_nm[2];
    normal_mode_fill_feedback(frame);
    frame->data6=frame->data7=0.0f; /* 控制模式1保留原有字段定义。 */
    /* data4/5为左右电机原始符号角度[度]，不是模型角度或IMU姿态。 */
    MotorFrame left, right;
    uint32_t now=imu_time_us();
    frame->data4=frame->data5=0.0f;
    if (motor_get_feedback(0U,&left) && left.mode==MOTOR_TORQUE &&
        !left.error_flag && (uint32_t)(now-g_motor_left.rx_timestamp_us)<=MOTOR_MAX_AGE_US &&
        isfinite(left.ang)) frame->data4=left.ang*57.29577951308232f;
    if (motor_get_feedback(1U,&right) && right.mode==MOTOR_TORQUE &&
        !right.error_flag && (uint32_t)(now-g_motor_right.rx_timestamp_us)<=MOTOR_MAX_AGE_US &&
        isfinite(right.ang)) frame->data5=right.ang*57.29577951308232f;
    (void)control_mode1_get_command(driver_nm);
    frame->data8=driver_nm[0];
    frame->data9=driver_nm[1];
}

