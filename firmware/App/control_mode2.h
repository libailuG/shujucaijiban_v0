#ifndef CONTROL_MODE2_H
#define CONTROL_MODE2_H
#include <stdint.h>
#include "mcu_uart_protocol.h"
enum { CONTROL2_OK=0, CONTROL2_STARTING=1, CONTROL2_DISABLED=2,
    CONTROL2_SUPPRESSED=3, CONTROL2_BAD_FEEDBACK=-1, CONTROL2_BAD_PERIOD=-2,
    CONTROL2_NUMERIC_ERROR=-3 };
typedef struct {
    int32_t status;
    uint32_t ticks, resets, last_compute_us, max_compute_us;
    uint32_t last_period_us, max_period_us, deadline_misses;
    float average_period_ms; /* Since last reset; zero until first timed cycle. */
    float average_compute_ms; /* Whole completed control computation; UART2 data6. */
    uint32_t guard_latched, oscillation_latched;
    float model_torque_nm[2], driver_torque_nm[2];
} ControlMode2Monitor;
extern volatile ControlMode2Monitor g_control_mode2_monitor;
extern volatile uint32_t g_control_mode2_torque_enabled;
/* Startup scale=0. UART9 mode2 data3 supplies a nonnegative finite scale.
 * UINT32_MAX denotes a direct scale rather than a legacy UI level. */
extern volatile uint32_t g_control_mode2_output_level;
extern volatile float g_control_mode2_output_scale;
int control_mode2_set_output_level(uint32_t level);
void control_mode2_set_output_scale(float scale);
void control_mode2_reset(void);
void control_mode2_set_torque_enabled(uint32_t enabled);
void control_mode2_poll(void);
int control_mode2_get_command(float driver_nm[2]);
void control_mode2_fill_feedback(volatile td_mcu_frame_t *frame);
#endif
