#ifndef BOARD_IMU_H
#define BOARD_IMU_H
#include <stdint.h>

/* 实板方向备注（用户确认）：板子平放时，X轴朝左，Y轴朝后。 */
typedef struct { float x, y, z; } ImuVector3;
typedef struct { float roll, pitch, yaw; } ImuAngles;
typedef struct {
    ImuAngles angle_rad;       /* ZYX Euler angles; yaw is relative/drifts. */
    ImuVector3 gyro_rad_s;
    ImuVector3 accel_m_s2;     /* Includes gravity. */
    uint32_t timestamp_us;     /* TIM2, wraps about every 71.6 minutes. */
    uint32_t sample_count;
    uint32_t valid;            /* Last transaction succeeded. */
    uint32_t angle_valid;      /* Gravity initialized the attitude estimate. */
    int32_t last_error;        /* 0=OK; negative=initialization/read failure. */
} ImuData;

extern volatile ImuData g_imu_data; /* 原传感器坐标，保留作对照。 */
/* 2026-11-13机身安装校正：X前/Y左/Z上。
 * 相对旧(原Z,原X,原Y)，机身X/Y同时反向、Z不变，即绕机身Z转180度。
 * (X,Y,Z)=(-原Z,-原X,原Y)，加速度m/s^2、角速度rad/s、姿态rad。
 * 加速度和陀螺同向变换后重新融合姿态；网络层保留原标准坐标转换。
 * Roll绕机身X、Pitch绕机身Y，右手正方向；Yaw为相对航向，会漂移。
 */
extern volatile ImuData g_imu_body_data;
void imu_get_body_data(ImuData *out); /* 一致快照；检查valid/angle_valid。 */
void imu_init(void);
void imu_poll(void);          /* Consume IRQ completions and solve attitude in main. */
void imu_get_data(ImuData *out);
#endif
