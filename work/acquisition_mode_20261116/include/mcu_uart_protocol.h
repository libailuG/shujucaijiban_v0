#ifndef TOUCHDESK_MCU_UART_PROTOCOL_H
#define TOUCHDESK_MCU_UART_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <float.h>
#include <limits.h>

#define TD_MCU_FRAME_SIZE 48u
#define TD_MCU_START UINT32_C(0xAAAAAAAA)
#define TD_MCU_END   UINT32_C(0xEEEEEEEE)

/* UART business frame. No ESP-NOW headers or CRC on this interface. */
typedef struct {
    uint32_t start_flag;
    uint32_t data1;
    uint32_t data2;
    float data3;
    float data4;
    float data5;
    float data6;
    float data7;
    float data8;
    float data9;
    float data10;
    uint32_t end_flag;
} td_mcu_frame_t;

/* C99-compatible compile-time layout checks; no compiler-specific packing. */
typedef char td_check_byte[(CHAR_BIT == 8) ? 1 : -1];
typedef char td_check_float[(sizeof(float) == 4 && FLT_RADIX == 2 &&
    FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128) ? 1 : -1];
typedef char td_check_size[(sizeof(td_mcu_frame_t) == 48) ? 1 : -1];
typedef char td_check_data1[(offsetof(td_mcu_frame_t, data1) == 4) ? 1 : -1];
typedef char td_check_data2[(offsetof(td_mcu_frame_t, data2) == 8) ? 1 : -1];
typedef char td_check_data3[(offsetof(td_mcu_frame_t, data3) == 12) ? 1 : -1];
typedef char td_check_data10[(offsetof(td_mcu_frame_t, data10) == 40) ? 1 : -1];
typedef char td_check_tail[(offsetof(td_mcu_frame_t, end_flag) == 44) ? 1 : -1];

static inline void td_mcu_put_u32(uint8_t *p, uint32_t v) {
    p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8);
    p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24);
}
static inline uint32_t td_mcu_get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1]<<8) |
           ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24);
}
static inline void td_mcu_put_float(uint8_t *p, float v) {
    uint32_t bits; memcpy(&bits, &v, 4); td_mcu_put_u32(p, bits);
}
static inline float td_mcu_get_float(const uint8_t *p) {
    uint32_t bits=td_mcu_get_u32(p); float v; memcpy(&v, &bits, 4); return v;
}

/* Encode fixed markers and all ten values into caller-owned 48-byte storage. */
static inline void td_mcu_encode(uint8_t out[TD_MCU_FRAME_SIZE],
                                 const td_mcu_frame_t *f) {
    td_mcu_put_u32(out, TD_MCU_START);
    td_mcu_put_u32(out+4, f->data1); td_mcu_put_u32(out+8, f->data2);
    td_mcu_put_float(out+12, f->data3); td_mcu_put_float(out+16, f->data4);
    td_mcu_put_float(out+20, f->data5); td_mcu_put_float(out+24, f->data6);
    td_mcu_put_float(out+28, f->data7); td_mcu_put_float(out+32, f->data8);
    td_mcu_put_float(out+36, f->data9); td_mcu_put_float(out+40, f->data10);
    td_mcu_put_u32(out+44, TD_MCU_END);
}

/* Input must contain at least 48 bytes. Framing only; no payload CRC. */
static inline int td_mcu_decode(td_mcu_frame_t *f,
                                const uint8_t in[TD_MCU_FRAME_SIZE]) {
    if (td_mcu_get_u32(in)!=TD_MCU_START ||
        td_mcu_get_u32(in+44)!=TD_MCU_END) return 0;
    f->start_flag=TD_MCU_START; f->end_flag=TD_MCU_END;
    f->data1=td_mcu_get_u32(in+4); f->data2=td_mcu_get_u32(in+8);
    f->data3=td_mcu_get_float(in+12); f->data4=td_mcu_get_float(in+16);
    f->data5=td_mcu_get_float(in+20); f->data6=td_mcu_get_float(in+24);
    f->data7=td_mcu_get_float(in+28); f->data8=td_mcu_get_float(in+32);
    f->data9=td_mcu_get_float(in+36); f->data10=td_mcu_get_float(in+40);
    return 1;
}

#endif

