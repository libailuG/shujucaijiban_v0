#ifndef RESERVED_UART2_H
#define RESERVED_UART2_H
#include <stdint.h>

/* Board JP7 "reserved serial 2" is UART9, not USART2.
 * JP7: 1=GND, 2=RX/PD14, 3=TX/PD15, 4=3.3V. */
#define RESERVED_UART2_BAUD 921600UL
#define RESERVED_UART2_HELLO_PERIOD_US 1000000UL
extern volatile uint32_t g_uart2_tx_count; /* Fully transmitted hello messages. */
extern volatile uint32_t g_uart2_rx_count;
extern volatile uint32_t g_uart2_last_rx;
extern volatile uint32_t g_uart2_error_count;
/* Call after clock setup and imu_init() (TIM2 1MHz timebase). */
void reserved_uart2_init(void);
void reserved_uart2_poll(void);
#endif
