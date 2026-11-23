#ifndef NORMAL_MODE_H
#define NORMAL_MODE_H
#include "mcu_uart_protocol.h"
/* 普通模式：电压、原始坐标Roll/Pitch(rad)，其余反馈清零。 */
void normal_mode_fill_feedback(volatile td_mcu_frame_t *frame);
#endif
