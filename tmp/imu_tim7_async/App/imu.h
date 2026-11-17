#ifndef BOARD_IMU_H
#define BOARD_IMU_H
#include <stdint.h>

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

extern volatile ImuData g_imu_data;
void imu_init(void);
void imu_poll(void);          /* Consume IRQ completions and solve attitude in main. */
void imu_get_data(ImuData *out);
#endif
