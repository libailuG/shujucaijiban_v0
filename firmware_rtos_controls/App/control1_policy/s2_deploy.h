/*
 * S2 DS v30 单片机部署接口与使用说明（UTF-8，2026-11-20）
 * 嵌入式接入文件三份：s2_deploy.c、s2_deploy.h、main.c（硬件接入示例）。
 * s2_deploy.c 已包含全部 FP32 权重、网络、历史、坐标转换和自检，无需其它模型文件。
 *
 * 【快速接入】
 * 1. 工程加入 s2_deploy.c，包含本头文件；main.c 是示例，可合并到现有主循环。
 * 2. 启动调用 s2_app_init()，填写 s2_board_config 的安装旋转、左右编码器零点、
 *    encoder_sign 和 motor_sign；完成实际标定后 calibration_confirmed=1。
 * 3. s2_app_config及底层config默认均用瞬时陀螺仪 S2_GYRO_SENSOR，
 *    与当前v30仿真一致。默认主动限幅12Nm，无限速，10次零输出暖机。
 * 4. 每20ms读取同步 S2HardwareInput 快照，measured_dt_s 填实测周期：
 *      float driver_nm[2];
 *      S2Status status = s2_app_tick_50hz(&snapshot, driver_nm);
 *    无论status如何都显式发送 driver_nm（包含零），不能故障时保留旧扭矩。
 *    发生通信失败应复位/禁用。示例只给关节端Nm，不包含HAL/CAN/UART/电流环。
 * 5. 可先用 s2_selftest(&workspace,&error) 做无电机网络自检，0为通过。
 *
 * 【IMU相对人体的朝向】
 * 人体解剖系A：+X前、+Y左、+Z上。模型腰带Waist系B：+X右、+Y前、+Z上。
 * 右/左均为穿戴者自身；直立零位 v_A=[v_B.y,-v_B.x,v_B.z]。
 * IMU必须跟随腰带，不是胸部。若传感器+Xs右/+Ys前/+Zs上：quat_sb=[1,0,0,0]。
 * 若传感器+Xs前/+Ys左/+Zs上：quat_sb=[0.7071067812,0,0,-0.7071067812]。
 * 其它安装方向须按实际轴标定，不能随意交换欧拉角名称。
 * quat_sb 将B向量转到传感器S：v_S=R_sb*v_B。
 * 输入quat_ws必须是Hamilton [w,x,y,z]、传感器S->Z朝上世界W：
 *   quat_wb=quat_ws*quat_sb；gyro_B=transpose(R_sb)*gyro_S。
 * NED、xyzw、世界->传感器的四元数应在驱动层先转换；姿态来自AHRS融合，
 * 不可把动态原始加速度直接当作重力。有效四元数模长0.9..1.1会自动归一化。
 *   g_B=transpose(R_wb)*[0,0,-1]
 *   obs[60]=atan2(-g_By,-g_Bz)/pi，原包称pitch，前倾为负；
 *   obs[61]=atan2( g_Bx,-g_Bz)/pi，原包称roll，右倾为正；
 *   obs[62..64]=gyro_B_xyz/(4*pi)。不直接接受驱动同名roll/pitch字段。
 *

 * 【v30完整65维obs索引表：输入网络标准化之前，全部为无量纲】
 * 历史h=0..9，索引基址b=6*h；h=0是t-180ms，h=9是当前t。
 * reset历史全零，每个有效20ms帧推进一次；不是用首帧补齐。
 * obs[b+0]：穿戴者左外骨骼髋角qL / pi；原始rad，后伸正/前抬负。
 * obs[b+1]：穿戴者右外骨骼髋角qR / pi；原始rad，后伸正/前抬负。
 * obs[b+2]：左髋dqL / (4*pi)；原始rad/s，后伸运动正/前抬运动负。
 * obs[b+3]：右髋dqR / (4*pi)；原始rad/s，后伸运动正/前抬运动负。
 * obs[b+4]：该历史帧采样前一周期的左主动命令 / 40；原始Nm，伸髋正。
 * obs[b+5]：该历史帧采样前一周期的右主动命令 / 40；原始Nm，伸髋正。
 *   40是模型训练的固定归一化尺度，不能改成当前12Nm输出上限。
 * obs[60]：atan2(-g_By,-g_Bz)/pi；原始rad，人体前倾负/后仰正。
 * obs[61]：atan2( g_Bx,-g_Bz)/pi；原始rad，人体右倾正/左倾负。
 * obs[62]：腰带角速度omega_Bx/(4*pi)，原始rad/s；+X向人体右，
 *   正旋转按右手定则，直立附近对应后仰角速度（前倾负）。
 * obs[63]：omega_By/(4*pi)，原始rad/s；+Y向人体前，
 *   正旋转按右手定则，直立附近对应右倾角速度（左倾负）。
 * obs[64]：omega_Bz/(4*pi)，原始rad/s；+Z向上，
 *   正旋转按右手定则，直立附近对应人体向左转（向右转负）。
 * 60/61是重力投影倾角，并非通用欧拉角；大倾角时不要按欧拉角叠加解释。
 * 62..64为当前瞬时体坐标陀螺仪，不是欧拉角导数，不是世界坐标角速度。
 * 网络内部再做 z[i]=(obs[i]-mean[i])/(std[i]+epsilon)，随后ReLU MLP。
 * 网络raw[0/1]是左右无界输出，无单位；clip(raw,-1,1)*12得到主动Nm。
 * 模型只接外骨骼传感器；不部署H170本体网络，不输入人体参考或人体力矩。
 * 本包软件验证不等于实机验证，无硬件通信；实际IMU安装/编码器/电机方向待标定。
 * gyro差分模式仅兼容诊断；v30默认及与当前sim2sim对照必须用SENSOR模式。
 *
 * 【左右编码器与电机方向】
 * 所有二元素数组：[0]穿戴者左髋，[1]穿戴者右髋；不是观察者左/右。
 * 模型Hip_left/right正轴都约朝人体左侧；正角/正速度/正扭矩约对应髋伸展
 * （腿向后），向前屈髋为负；人体L/R_HipFlex正方向相反，不能混用。
 *   q_model=encoder_sign*(encoder_joint_rad-encoder_zero_rad)
 *   qd_model=encoder_sign*encoder_joint_rad_s
 * encoder_sign每侧独立，只能+1/-1；角度速度同符号。零点对应模型机械q=0，
 * 不能未经机构对齐把任意站立姿态自动置零。跨圈信号在驱动层先展开。
 *   driver_joint_Nm=motor_sign*active_model_Nm
 * motor_sign每侧独立台架标定，不假定等于encoder_sign或左右相同。
 * 正模型扭矩应对大腿施加伸展力矩，对腰部产生反力矩。
 * 原绘图的乘-1是人体比较符号，不是硬件电机方向配置！
 * 编码器输入、扭矩输出均为输出关节端；转子端数据先按齿比/机构换算。
 * 电流驱动还需核对Kt、减速比、效率、峰值/RMS电流和协议缩放；本包不猜参数。
 * 单位：角rad、角速度rad/s、扭矩Nm；deg*pi/180；rpm*2*pi/60。
 *
 * 【策略时序与反馈】
 * 50Hz固定周期，measured_dt_s允许20ms +/-1ms；超出则零输出并清空状态。
 * 前10次有效使能更新历史但零主动命令，暖机仍计算网络原始输出但主动命令保持零，第11次开始输出。禁用/错误重新暖机。
 * 65维：最旧在前10帧，每帧[qL/pi,qR/pi,qdL/(4pi),qdR/(4pi),prevL/40,prevR/40]，
 * 加当前5维IMU。mean/std/epsilon和权重已在.c中；最后一层无激活。
 * 主动命令=clip(raw,-1,1)*12Nm；可选torque_limit_nm及slew_rate_nm_s会改变包装。
 * prev是上一周期处理后的模型坐标主动命令，不是电流/实测扭矩/驱动符号命令。
 * 外部追加主动限幅时，发送成功后用s2_controller_commit_command反馈受限值；
 * 高级接口使用自行维护的S2Controller，而非示例的内部单例状态。
 * 发送失败则禁用并复位；驱动层另做新鲜度/超时/左右同步/看门狗。
 * S2_GYRO_QUAT_DIFFERENCE按100*vec(conj(q_prev)*q_now)计算并处理q/-q连续性；
 * S2_GYRO_SENSOR用当前真实体坐标陀螺仪，两种模式的噪声/延迟不同，需分开实测。
 * 仿真髋总力矩每1ms为clip(active-0.05*qd,-12,12)；本包输出仅主动命令，
 * 没有重复添加物理阻尼。若快速环实现该虚拟阻尼，不要将总扭矩反馈进历史。
 *
 * 【接入前核对】
 * 电机禁用时确认：站立g_B约[0,0,-1]；前倾obs60负、右倾obs61正；
 * 手动向前抬左右腿时各自q/qd为负；机械零点、左右通道正确。
 * 再在低扭矩台架上分别确认motor_sign，记录标定值。默认配置均是占位值，
 * calibration_confirmed=0时拒绝硬件使能，真实安装参数尚未提供。
 *
 * 【编译与资源】
 * C99，链接libm(-lm)，不使用malloc。权重常量233236字节，必须放Flash/.rodata。
 * 每次57728乘加；S2Controller2068字节、S2Output284字节（当前M4F/PC ABI）。
 * 示例单例配置及状态BSS2412字节，另计调用栈、libm及外设驱动。
 * M4F示例参数：-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
 * -O2 -std=c99 -ffp-contract=off；匹配工程ABI、启动时启用FPU。不要-fast-math，
 * 以保留NaN/Inf检查。已做PC对比及交叉编译；未烧录，板端最坏耗时仍需实测。
 * 单例s2_app_*不重入；多实例请用各自S2Controller和工作区，勿在中断并发调用。
 */
#ifndef S2_DEPLOY_H
#define S2_DEPLOY_H
#include <stdint.h>
#include <float.h>
#define S2_DEPLOY_VERSION "v30_stm32f405_12nm_sensor_50hz"
#define S2_DEPLOY_ABI 30
#if FLT_RADIX != 2 || FLT_MANT_DIG != 24 || FLT_MAX_EXP != 128
#error "Require IEEE binary32 float"
#endif
#ifdef __cplusplus
extern "C" {
#endif

/* ---- Network API ---- */
#define S2_OBS_DIM 65
#define S2_ACTION_DIM 2
/* One workspace per concurrent caller; no heap allocation. */
typedef struct { float normalized[65], a[256], b[128]; } S2PolicyWorkspace;
/* Returns 0 on success, -1 for invalid/nonfinite input or result.
 * obs is PRE-normalization, raw is unbounded [left,right]. */
int s2_policy_forward(const float obs[65], float raw[2], S2PolicyWorkspace *work);

/* ---- Stateful controller API ---- */
#define S2_DT_S 0.02f
#define S2_PI 3.14159265358979323846f
typedef enum { S2_GYRO_QUAT_DIFFERENCE = 0, S2_GYRO_SENSOR = 1 } S2GyroMode;
typedef enum { S2_OK = 0, S2_WARMUP = 1, S2_DISABLED = 2,
               S2_BAD_INPUT = -1, S2_BAD_CONFIG = -2, S2_NUMERIC_ERROR = -3 } S2Status;
typedef struct {
    S2GyroMode gyro_mode;
    float torque_limit_nm; /* Active command: 0..12; default 12. */
    float slew_rate_nm_s;  /* 0 = no slew limit (source behavior). */
    uint32_t warmup_ticks; /* Default 10. */
} S2Config;
/* All inputs ALREADY in model coordinates, synchronized at 50 Hz.
 * B=Waist: +X right, +Y forward, +Z up at neutral posture.
 * quat_wb is Hamilton wxyz, maps B vectors into a Z-up world frame.
 * hip[0:2] are Hip_left/right, positive approximately EXTENSION, not
 * human L/R_HipFlex (which are positive FLEXION). No automatic zeroing.
 * gyro_body_rad_s is expressed in B (only required in SENSOR mode).
 * measured_dt_s must be 0.02 +/- 0.001 s; caller checks timestamps/staleness.
 */
typedef struct {
    float hip_rad[2], hip_rad_s[2], quat_wb[4], gyro_body_rad_s[3];
    float measured_dt_s;
    uint32_t enabled, sensors_valid;
} S2Input;
typedef struct {
    float obs[65]; /* 10x[qL/pi,qR/pi,qdL/4pi,qdR/4pi,prevL/40,prevR/40],
                    oldest first; pitch/pi,roll/pi,gyro_B_xyz/4pi.
                    pitch=atan2(-gBy,-gBz), roll=atan2(gBx,-gBz). */
    float raw[2], requested_nm[2], command_nm[2];
} S2Output;
typedef struct {
    float history[60], previous_quat[4], previous_command[2];
    uint32_t ticks, quat_initialized;
    S2PolicyWorkspace work;
} S2Controller;
void s2_config_default(S2Config *config);
void s2_controller_reset(S2Controller *state);
S2Status s2_controller_step(S2Controller *state, const S2Config *config,
                            const S2Input *input, S2Output *output);
/* Call after an EXTERNAL active-command limit, in model Nm BEFORE motor sign.
 * Never feed measured torque/current or passive damping torque into history.
 * On disable/fault reset instead. Returns -1 for nonfinite or abs >12. */
int s2_controller_commit_command(S2Controller *state, const float active_model_nm[2]);

/* ---- Hardware coordinate API ---- */
/* HARDWARE CALIBRATION REQUIRED. No CAN/UART/HAL assumptions.
 * B = model Waist: +X RIGHT, +Y FORWARD, +Z UP (right-handed).
 * A = anatomical frame: +X FORWARD, +Y LEFT, +Z UP.
 * At neutral: v_A = [v_B.y, -v_B.x, v_B.z].
 * A physically aligned IMU has +Xs right,+Ys forward,+Zs up.
 * Otherwise q_sb maps B vectors TO sensor frame S: v_S=R(q_sb)*v_B.
 * AHRS must provide q_ws (sensor -> Z-up world), Hamilton wxyz.
 * Then q_wb=q_ws*q_sb; gyro_B=R(q_sb)^T*gyro_S.
 * If sensor axes are anatomical (+Xs forward,+Ys left,+Zs up), use
 * q_sb=[sqrt(1/2),0,0,-sqrt(1/2)], NOT identity.
 * Never pass NED/world->sensor/xyzw data without conversion.
 *
 * Index 0 = wearer's LEFT, 1 = wearer's RIGHT (not observer's left).
 * hip_model = encoder_sign*(encoder_joint_rad - encoder_zero_rad).
 * velocity uses SAME encoder_sign. Both hips: positive approximately
 * EXTENSION (leg backwards), negative FLEXION (leg forwards).
 * Model +torque applies extension torque to thigh, reaction to waist.
 * Driver positive direction is UNKNOWN until bench-calibrated, separately
 * on each motor. driver_joint_Nm=motor_sign*model_active_Nm.
 * Do not copy the plotting '-1' or assume left/right motor signs match.
 * encoder inputs and driver outputs are JOINT-side, not rotor-side.
 */
typedef struct {
    float quat_sb[4];
    float encoder_sign[2], encoder_zero_rad[2], motor_sign[2];
    uint32_t calibration_confirmed; /* Default 0: conversion refuses enable. */
} S2PortConfig;
typedef struct {
    float encoder_joint_rad[2], encoder_joint_rad_s[2];
    float quat_ws[4], gyro_sensor_rad_s[3], measured_dt_s;
    uint32_t enabled, sensors_valid;
} S2HardwareInput;
void s2_port_config_default(S2PortConfig *config);
/* -1 = uncalibrated/invalid; output remains disabled and zero. */
int s2_port_map_input(const S2PortConfig *config, const S2HardwareInput *hw, S2Input *model);
/* Active-only JOINT-side Nm, no current/gear conversion or passive torque.
 * MUST check return and send explicit ZERO on any fault/disable; never keep
 * previous nonzero command when controller status is negative or disabled.
 */
int s2_port_map_output(const S2PortConfig *config, const float active_model_nm[2],
                       float driver_joint_nm[2]);

/* ---- Simple singleton application API ---- */
void s2_app_init(void);
/* Returns explicit joint-side driver commands; caller must transmit every tick,
 * including zeros. Driver watchdog/fast current loop remains board-owned. */
S2Status s2_app_tick_50hz(const S2HardwareInput *input, float driver_joint_nm[2]);
/* Fill calibrated fields after init; defaults deliberately unconfirmed. */
extern S2PortConfig s2_board_config;
extern S2Config s2_app_config;

/* ---- Startup self-test API ---- */
/* Optional startup test. 0=pass, -1=fail. Compare raw float32 outputs.
 * No motor output; workspace supplied by caller. Report max absolute error. */
int s2_selftest(S2PolicyWorkspace *workspace, float *max_error);

#ifdef __cplusplus
}
#endif
#endif
