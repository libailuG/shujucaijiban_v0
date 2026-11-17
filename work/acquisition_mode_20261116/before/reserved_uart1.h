#ifndef RESERVED_UART1_H
#define RESERVED_UART1_H
#include <stdint.h>
#define RESERVED_UART1_BAUD 921600UL
#include "control_mode1_monitor.h"
void reserved_uart1_init(void);
void reserved_uart1_poll(void);
/* Main-loop only; copies snapshot, returns 0 if busy or outside mode1. */
int reserved_uart1_send_monitor(const ControlMode1Monitor *frame);
extern volatile uint32_t g_uart1_tx_drop_count;
extern volatile uint32_t g_uart1_tx_count;
#endif
