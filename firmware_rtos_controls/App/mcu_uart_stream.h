#ifndef MCU_UART_STREAM_H
#define MCU_UART_STREAM_H
#include "mcu_uart_protocol.h"

typedef struct {
    uint8_t bytes[TD_MCU_FRAME_SIZE];
    uint32_t used;
    uint32_t bad_tail_count;
    uint32_t discarded_bytes;
} McuUartStream;

/* Called only by the main-loop consumer. Payload markers are allowed. */
static inline int mcu_uart_stream_push(McuUartStream *s, uint8_t byte,
                                       td_mcu_frame_t *frame)
{
    if (s->used < 4U) {
        if (byte == 0xAAU) s->bytes[s->used++] = byte;
        else { s->discarded_bytes += s->used + 1U; s->used = 0U; }
        return 0;
    }
    s->bytes[s->used++] = byte;
    if (s->used < TD_MCU_FRAME_SIZE) return 0;
    if (td_mcu_decode(frame, s->bytes)) { s->used = 0U; return 1; }
    ++s->bad_tail_count;
    /* Invalid candidate: discard just its first byte, then search again.
     * Keep a full later header or at most three trailing AA bytes. */
    do {
        --s->used;
        memmove(s->bytes, s->bytes + 1, s->used);
        ++s->discarded_bytes;
    } while (s->used >= 4U && td_mcu_get_u32(s->bytes) != TD_MCU_START);
    if (s->used < 4U) {
        uint32_t keep = 0U;
        while (keep < s->used && s->bytes[s->used - 1U - keep] == 0xAAU) ++keep;
        s->discarded_bytes += s->used - keep;
        s->used = keep; /* All kept bytes are AA, so their positions do not matter. */
    }
    return 0;
}
#endif
