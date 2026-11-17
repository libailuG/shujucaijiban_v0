#ifndef RS485_LINK_TEST_H
#define RS485_LINK_TEST_H
#include <stdint.h>
#define RS485_LINK_TEST_BAUD 3000000UL
/* Board-only test: JP2 A<->JP3 A, B<->B, GND<->GND. Leave pin4 unconnected.
 * Disconnect motors. This test replaces motor_bus_init/poll in main. */
typedef struct {
    uint32_t tx_count, rx_count, rx_bytes, last_sequence, rx_timestamp_us;
    uint32_t bad_frame, sequence_errors, uart_errors, overflows, busy_skips;
    uint32_t error_flags, valid;
    uint8_t last_rx[48];
} Rs485LinkStats;
extern volatile Rs485LinkStats g_rs485_jp2, g_rs485_jp3;
/* Bit0 JP3->JP2 received, bit1 JP2->JP3 received; sticky, not a live health flag. */
extern volatile uint32_t g_rs485_link_pass;
void rs485_link_test_init(void);
void rs485_link_test_poll(void);
#endif
