#ifndef RESERVED_UART2_H
#define RESERVED_UART2_H
#include <stdint.h>
#include "mcu_uart_protocol.h"

/* Board JP7 "reserved serial 2" is UART9, not USART2.
 * JP7: 1=GND, 2=RX/PD14, 3=TX/PD15, 4=3.3V. */
#define RESERVED_UART2_STATIONARY_MASK (1UL << 3) /* 按钮4：为1时强制选择普通静止模式data2=0。 */
#define RESERVED_UART2_ACQUISITION_VALUE 20U
#define RESERVED_UART2_DEBUG_MODE_VALUE 11U /* data2=11：电机调试。 */
#define RESERVED_UART2_IMU_MODE_VALUE 12U /* data2=12：IMU调试。 */
#define RESERVED_UART2_CONTROL_MODE1_VALUE 1U /* data2整个uint32恰好为1：控制模式1。 */
#define RESERVED_UART2_LEFT_THIGH_VALUE 13U
#define RESERVED_UART2_LEFT_SHANK_VALUE 14U
#define RESERVED_UART2_LEFT_PRESSURE_VALUE 15U
#define RESERVED_UART2_RIGHT_THIGH_VALUE 16U
#define RESERVED_UART2_RIGHT_SHANK_VALUE 17U
#define RESERVED_UART2_RIGHT_PRESSURE_VALUE 18U
/* Uplink validity of the selected sensor payload. */
#define RESERVED_UART2_DATA_VALID_MASK (1UL << 6)
#define RESERVED_UART2_ATTITUDE_VALID_MASK (1UL << 7)
#define RESERVED_UART2_SENSOR_STATUS_MASK (RESERVED_UART2_DATA_VALID_MASK | RESERVED_UART2_ATTITUDE_VALID_MASK)
static inline uint32_t reserved_uart2_accept_mode(uint32_t value) {
    return (value == RESERVED_UART2_CONTROL_MODE1_VALUE || value == RESERVED_UART2_ACQUISITION_VALUE ||
        (value >= RESERVED_UART2_DEBUG_MODE_VALUE && value <= RESERVED_UART2_RIGHT_PRESSURE_VALUE)) ? value : 0U;
}
#include "control_mode1.h" /* 兼容原入口声明，实现在独立模块。 */
extern volatile uint32_t g_uart2_debug_mode; /* 实际已接受模式0/1/11..18/20，上行data2回执。 */
#define RESERVED_UART2_CONTROL1_TORQUE_MASK (1UL << 0) /* 仅模式1：策略扭矩下发开关，默认关。 */
#define RESERVED_UART2_ZERO_SWITCH_MASK (1UL << 0) /* data1.bit0：置零请求/执行闪烁/恢复完成。 */
#define RESERVED_UART2_BAUD 921600UL
#define RESERVED_UART2_TX_PERIOD_US 50000UL /* 50ms = 20Hz. */
extern volatile td_mcu_frame_t g_uart2_tx_frame;
extern volatile td_mcu_frame_t g_uart2_rx_frame;
extern volatile uint32_t g_uart2_tx_count; /* Locally completed binary frames. */
extern volatile uint32_t g_uart2_rx_count;
extern volatile uint32_t g_uart2_last_rx;
extern volatile uint32_t g_uart2_error_count;
extern volatile uint32_t g_uart2_rx_frame_count;
extern volatile uint32_t g_uart2_rx_bad_tail_count;
extern volatile uint32_t g_uart2_rx_overflow_count;
extern volatile uint32_t g_uart2_rx_timestamp_us;
extern volatile uint32_t g_uart2_rx_valid; /* A complete frame has been received. */
/* Call after clock setup and imu_init() (TIM2 1MHz timebase). */
void reserved_uart2_init(void);
void reserved_uart2_poll(void);
/* Latest received frame only; returns 0 until the first valid frame. */
int reserved_uart2_get_rx(td_mcu_frame_t *out);
/* Main-loop API: takes a copy; fixed start/end flags are applied on encode. */
void reserved_uart2_set_tx(const td_mcu_frame_t *frame);
#endif




