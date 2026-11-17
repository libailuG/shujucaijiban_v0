#ifndef CASCADE_LINK_H
#define CASCADE_LINK_H
#include "cascade_receiver.h"
enum { CASCADE_LISTEN, CASCADE_WAIT_INFO, CASCADE_WAIT_START, CASCADE_STREAMING, CASCADE_RETRY };
typedef struct {
    uint32_t state, rx_bytes, uart_errors, dma_errors, overruns, partial_timeouts;
    uint32_t tx_requests, tx_errors, bad_replies, bad_crc, bad_length;
    uint32_t accepted, rejected, duplicate_push, skipped_push, bound;
    uint32_t new_samples[3];
} CascadeDiagnostics;
#endif

