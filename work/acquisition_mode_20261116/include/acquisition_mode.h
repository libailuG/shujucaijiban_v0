#ifndef ACQUISITION_MODE_H
#define ACQUISITION_MODE_H
#include "mcu_uart_protocol.h"
#define ACQUISITION_PERIOD_US 10000U
#define ACQUISITION_FRAME_SIZE 80U
/* Wire order; flags are uint32, all physical quantities float32. */
typedef struct {
    uint32_t start_flag, timestamp_us;
    float left_motor_angle, left_motor_speed, right_motor_angle, right_motor_speed;
    float body_gyro_x, body_gyro_y, body_accel_z;
    float left_thigh_angle, left_thigh_speed, left_shank_angle, left_shank_speed;
    uint32_t left_pressure;
    float right_thigh_angle, right_thigh_speed, right_shank_angle, right_shank_speed;
    uint32_t right_pressure, end_flag;
} AcquisitionFrame;
typedef char acquisition_layout[(sizeof(AcquisitionFrame)==80 &&
    offsetof(AcquisitionFrame,left_pressure)==52 && offsetof(AcquisitionFrame,right_pressure)==72 &&
    offsetof(AcquisitionFrame,end_flag)==76)?1:-1];
void acquisition_mode_set_enabled(uint32_t enabled);
uint32_t acquisition_mode_is_enabled(void);
void acquisition_mode_poll(void);
void acquisition_mode_fill_feedback(volatile td_mcu_frame_t *frame);
/* Main-loop snapshot; returns bit6/7 aggregate validity for UART2 fields. */
uint32_t acquisition_mode_snapshot(AcquisitionFrame *out);
void acquisition_mode_encode(uint8_t out[ACQUISITION_FRAME_SIZE], const AcquisitionFrame *frame);
#endif


