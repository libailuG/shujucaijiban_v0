#ifndef CONTROL_MODE3_H
#define CONTROL_MODE3_H
#include <stdint.h>
#include "mcu_uart_protocol.h"
enum { CONTROL3_OK=0, CONTROL3_DISABLED=1, CONTROL3_ZEROING=2,
 CONTROL3_LEFT_INVALID=4, CONTROL3_RIGHT_INVALID=8, CONTROL3_LEFT_OLD=16,
 CONTROL3_RIGHT_OLD=32, CONTROL3_SKEW=64, CONTROL3_LEFT_MODE=128,
 CONTROL3_RIGHT_MODE=256, CONTROL3_LEFT_ERROR=512, CONTROL3_RIGHT_ERROR=1024,
 CONTROL3_LEFT_NUMERIC=2048, CONTROL3_RIGHT_NUMERIC=4096 };
typedef struct { uint32_t status, faults, resets, elapsed_us; float torque_nm[2]; } ControlMode3Monitor;
extern volatile ControlMode3Monitor g_control_mode3_monitor;
extern volatile uint32_t g_control_mode3_torque_enabled;
/* Valid amplitude 0..15 Nm, period 0.1..60 s. Changes restart at zero. */
int control_mode3_set_parameters(float amplitude_nm, float period_s);
extern volatile float g_control_mode3_amplitude_nm, g_control_mode3_period_s;
extern volatile uint32_t g_control_mode3_parameter_rejects;
void control_mode3_reset(void);
void control_mode3_set_torque_enabled(uint32_t enabled);
void control_mode3_poll(void);
int control_mode3_get_command(float nm[2]);
void control_mode3_fill_feedback(volatile td_mcu_frame_t *frame);
#endif
