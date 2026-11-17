#ifndef IMU_DEBUG_MODE_H
#define IMU_DEBUG_MODE_H
#include "mcu_uart_protocol.h"
/* IMU调试：使用已确认的竖装机身坐标，角度为度，不含yaw。 */
void imu_debug_mode_fill_feedback(volatile td_mcu_frame_t *frame);
void cascade_debug_mode_fill_feedback(volatile td_mcu_frame_t *frame, uint32_t mode);
#endif

