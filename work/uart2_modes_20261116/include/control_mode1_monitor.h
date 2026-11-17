#ifndef CONTROL_MODE1_MONITOR_H
#define CONTROL_MODE1_MONITOR_H

#include <stdint.h>

#define CONTROL_MODE1_MONITOR_START_FLAG 0xAAAAAAAAUL
#define CONTROL_MODE1_MONITOR_END_FLAG   0xEEEEEEEEUL
#define CONTROL_MODE1_MONITOR_VERSION    1U

#define CONTROL_MODE1_MONITOR_LEFT_VALID    (1UL << 0)
#define CONTROL_MODE1_MONITOR_RIGHT_VALID   (1UL << 1)
#define CONTROL_MODE1_MONITOR_IMU_VALID     (1UL << 2)
#define CONTROL_MODE1_MONITOR_OBS_VALID     (1UL << 3)
#define CONTROL_MODE1_MONITOR_NETWORK_VALID (1UL << 4)
#define CONTROL_MODE1_MONITOR_COMMAND_VALID (1UL << 5)

/* All two-element arrays: [0] wearer's left, [1] wearer's right.
 * FP32 fields. Only the newest observation is monitored; inference still
 * uses the original 65-dimensional history. UART4: little-endian FP32, 160 bytes. */
typedef struct {
    uint32_t start_flag;
    uint32_t version;
    uint32_t frame_size;              /* sizeof(ControlMode1Monitor), bytes */
    uint32_t sequence;
    uint32_t timestamp_us;
    int32_t status;                    /* 0 OK, 1 warmup, 2 disabled, <0 error */
    uint32_t valid_flags;
    uint32_t compute_time_us;
    float dt_s;

    /* Hardware input, before model direction/coordinate conversion. */
    float joint_angle_rad[2];
    float joint_speed_rad_s[2];
    float body_quat[4];               /* Hamilton [w,x,y,z], body -> world */
    float body_gyro_rad_s[3];

    /* Latest 11 observation values, BEFORE network mean/std normalization.
     * These use model coordinates, unlike the hardware input above. */
    float joint_angle_norm[2];        /* q / pi */
    float joint_speed_norm[2];        /* qd / (4*pi) */
    float previous_torque_norm[2];    /* previous model command / 40 */
    float body_tilt_norm[2];          /* model pitch/pi, roll/pi */
    float body_gyro_norm[3];          /* model XYZ gyro / (4*pi) */

    float raw_action[2];              /* Dimensionless network output */
    float requested_torque_nm[2];     /* clip(raw,-1,1) * 20 */
    float model_torque_nm[2];         /* Limited model-coordinate command */
    float driver_torque_nm[2];        /* Final driver command, not measured torque */
    uint32_t end_flag;
} ControlMode1Monitor;
typedef char ControlMode1MonitorSizeCheck[(sizeof(ControlMode1Monitor) == 160U) ? 1 : -1];

/* Published once per policy tick; invalid input reports are limited to 50Hz. */
extern volatile ControlMode1Monitor g_control_mode1_monitor;

#endif

