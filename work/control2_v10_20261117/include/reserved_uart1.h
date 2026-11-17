#ifndef RESERVED_UART1_H
#define RESERVED_UART1_H
#include <stdint.h>
#define RESERVED_UART1_BAUD 921600UL
#include "control_mode1_monitor.h"
#include "control_mode1_observation.h"
#include "acquisition_mode.h"
#include "control_mode2_telemetry.h"
int reserved_uart1_send_control2(const ControlMode2Telemetry *frame);
int reserved_uart1_send_acquisition(const uint8_t frame[ACQUISITION_FRAME_SIZE]);
void reserved_uart1_init(void);
void reserved_uart1_poll(void);
/* Main-loop only; copies snapshot, returns 0 if busy or upload is disabled. */
int reserved_uart1_send_observation(const ControlMode1Observation *frame);
extern volatile uint32_t g_uart1_tx_drop_count;
extern volatile uint32_t g_uart1_tx_count;
#endif

