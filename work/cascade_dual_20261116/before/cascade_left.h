#ifndef CASCADE_LEFT_H
#define CASCADE_LEFT_H
#include "cascade_receiver.h"
enum { CASCADE_LISTEN, CASCADE_WAIT_INFO, CASCADE_WAIT_START, CASCADE_STREAMING, CASCADE_RETRY };
typedef struct {
    uint32_t state, rx_bytes, uart_errors, dma_errors, overruns, partial_timeouts;
    uint32_t tx_requests, tx_errors, bad_replies, bad_crc, bad_length;
    uint32_t accepted, rejected, duplicate_push, skipped_push, bound;
    uint32_t new_samples[3];
} CascadeDiagnostics;
extern volatile CascadeDiagnostics g_cascade_left_diag;
void cascade_left_init(void);
void cascade_left_poll(void);
/* Safe from main or interrupts; independent of existing control inputs. */
void cascade_left_get_snapshot(CascadeSnapshot *out);
#endif
