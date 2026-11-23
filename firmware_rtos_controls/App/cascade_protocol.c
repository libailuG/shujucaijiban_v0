#include "cascade_protocol.h"
#include <string.h>
uint16_t cp_crc(const uint8_t *p, uint16_t n)
{
    static const uint16_t table[16]={0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
        0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef};
    uint16_t crc=0xffffU;
    while(n--) {
        uint8_t b=*p++;
        crc=(uint16_t)((crc<<4)^table[((crc>>12)^(b>>4))&15U]);
        crc=(uint16_t)((crc<<4)^table[((crc>>12)^b)&15U]);
    }
    return crc;
}
uint16_t cp_encode(uint8_t *out,uint8_t type,uint32_t seq,const void *payload,uint16_t n)
{
    if(n>CP_MAX_PAYLOAD || (n && !payload)) return 0;
    memset(out,0xaa,4); out[4]=CP_VERSION; out[5]=type;
    out[6]=(uint8_t)n;out[7]=(uint8_t)(n>>8);
    for(unsigned i=0;i<4;i++) out[8+i]=(uint8_t)(seq>>(8*i));
    if(n) memcpy(out+12,payload,n);
    uint16_t crc=cp_crc(out,(uint16_t)(12+n));
    out[12+n]=(uint8_t)crc;out[13+n]=(uint8_t)(crc>>8);
    return (uint16_t)(14+n);
}
static void discard(CpParser *p,unsigned n) { p->used=(uint16_t)(p->used-n); memmove(p->bytes,p->bytes+n,p->used); }
void cp_parser_reset(CpParser *p) { p->used=0; }
int cp_feed(CpParser *p,uint8_t byte,CpPacket *out)
{
    if(p->used==CP_MAX_FRAME) { discard(p,1); ++p->discarded; }
    p->bytes[p->used++]=byte;
    while(p->used>=4) {
        if(p->bytes[0]!=0xaa || p->bytes[1]!=0xaa || p->bytes[2]!=0xaa || p->bytes[3]!=0xaa) { discard(p,1); ++p->discarded; continue; }
        if(p->used<12) return 0;
        uint16_t n=(uint16_t)(p->bytes[6] | ((uint16_t)p->bytes[7]<<8));
        if(p->bytes[4]!=CP_VERSION || n>CP_MAX_PAYLOAD) { ++p->bad_length;discard(p,1);continue; }
        if(p->used<n+14U) return 0;
        uint16_t crc=(uint16_t)(p->bytes[12+n] | ((uint16_t)p->bytes[13+n]<<8));
        if(cp_crc(p->bytes,(uint16_t)(12+n))!=crc) { ++p->bad_crc;discard(p,1);continue; }
        out->type=p->bytes[5];out->length=n;out->sequence=0;
        for(unsigned i=0;i<4;i++) out->sequence|=(uint32_t)p->bytes[8+i]<<(8*i);
        if(n) memcpy(out->payload,p->bytes+12,n);
        discard(p,n+14U);return 1;
    }
    return 0;
}
uint32_t cp_age(uint32_t age,uint32_t elapsed) { return UINT32_MAX-age<elapsed ? UINT32_MAX:age+elapsed; }
void cp_age_record(CpRecord *r,uint32_t elapsed) { r->age_us=cp_age(r->age_us,elapsed); if(r->age_us>CP_FRESH_US) r->flags&=~(CP_VALID|CP_ATTITUDE_VALID); }
uint32_t cp_role_for_child(uint32_t child) { return child==CP_FOOT?CP_SHANK:(child==CP_SHANK?CP_THIGH:CP_UNKNOWN); }
