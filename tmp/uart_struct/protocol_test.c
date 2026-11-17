#include "App/mcu_uart_stream.h"
#define CHECK(c) do { if (!(c)) return __LINE__; } while (0)

uint32_t protocol_test(void)
{
    td_mcu_frame_t f = {0}, out = {0};
    uint8_t raw[49] = {0};
    static const uint8_t expected[48] = {
        0xAA,0xAA,0xAA,0xAA,5,0,0,0,1,0,0,0,0,0,0x80,0x3F,
        0,0,0,0xC0,0,0,0x60,0x40,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0xEE,0xEE,0xEE,0xEE};
    f.data1 = 5U; f.data2 = 1U;
    f.data3 = 1.0f; f.data4 = -2.0f; f.data5 = 3.5f;
    td_mcu_encode(raw + 1, &f); /* Deliberately unaligned storage. */
    CHECK(memcmp(raw + 1, expected, 48) == 0);
    CHECK(td_mcu_decode(&out, raw + 1));
    CHECK(out.data1 == 5U && out.data2 == 1U && out.data3 == 1.0f);
    CHECK(out.data4 == -2.0f && out.data5 == 3.5f);
    McuUartStream s = {0};
    /* Prefix junk, including broken headers and a header split byte by byte. */
    const uint8_t junk[] = {0,0xAA,1,0xAA,0xAA,2,0xEE,0xAA};
    for (uint32_t i = 0; i < sizeof(junk); ++i)
        CHECK(!mcu_uart_stream_push(&s, junk[i], &out));
    uint32_t frames = 0U;
    /* Consecutive frames, no idle gaps or external boundaries. */
    for (uint32_t n = 0; n < 3U; ++n)
        for (uint32_t i = 0; i < 48U; ++i)
            frames += (uint32_t)mcu_uart_stream_push(&s, expected[i], &out);
    CHECK(frames == 3U && out.data2 == 1U);
    /* Payload may contain exact frame markers. */
    f.data1 = TD_MCU_START; f.data2 = TD_MCU_END;
    uint32_t bits = TD_MCU_END;
    memcpy(&f.data3, &bits, 4);
    td_mcu_encode(raw, &f);
    for (uint32_t i = 0; i < 48U; ++i)
        frames += (uint32_t)mcu_uart_stream_push(&s, raw[i], &out);
    CHECK(frames == 4U && out.data1 == TD_MCU_START && out.data2 == TD_MCU_END);
    /* Bad tail followed by valid frame: discard one candidate byte, resync. */
    raw[47] = 0;
    for (uint32_t i = 0; i < 48U; ++i)
        frames += (uint32_t)mcu_uart_stream_push(&s, raw[i], &out);
    CHECK(frames == 4U);
    for (uint32_t i = 0; i < 48U; ++i)
        frames += (uint32_t)mcu_uart_stream_push(&s, expected[i], &out);
    CHECK(frames == 5U && s.bad_tail_count != 0U);
    /* A truncated candidate must not hide the next full frame. */
    for (uint32_t i = 0; i < 19U; ++i)
        CHECK(!mcu_uart_stream_push(&s, expected[i], &out));
    for (uint32_t i = 0; i < 48U; ++i)
        frames += (uint32_t)mcu_uart_stream_push(&s, expected[i], &out);
    CHECK(frames == 6U && s.used == 0U);
    /* Hundreds of leading AA bytes exercise overlapping header candidates. */
    for (uint32_t i = 0; i < 200U; ++i)
        CHECK(!mcu_uart_stream_push(&s, 0xAA, &out));
    for (uint32_t i = 0; i < 48U; ++i)
        frames += (uint32_t)mcu_uart_stream_push(&s, expected[i], &out);
    CHECK(frames == 7U && s.used == 0U);
    raw[0] = 0;
    CHECK(!td_mcu_decode(&out, raw));
    return 0;
}
