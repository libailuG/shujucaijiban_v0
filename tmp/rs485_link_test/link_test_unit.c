#include "App/rs485_link_test.c"
uint32_t imu_time_us(void) { return 12345U; }
#define CHECK(x) do { if(!(x)) return __LINE__; } while(0)
static void inject(LinkPort *p,const uint8_t *f,unsigned n) {
    for(unsigned i=0;i<n;i++) { p->ring[p->head]=f[i]; p->head=(p->head+1)&MASK; }
    consume(p);
}
int test_main(void) {
    /* Direct emulator entry bypasses the C runtime RW decompressor. */
    jp2.stats=&g_rs485_jp2; jp2.number=2;
    jp3.stats=&g_rs485_jp3; jp3.number=3;
    uint8_t f[48];
    make_frame(f,3,1);
    CHECK(memcmp(f,"TEST",4)==0 && get32(f+4)==3 && get32(f+8)==1);
    inject(&jp2,f,7); CHECK(!g_rs485_jp2.valid);
    inject(&jp2,f+7,41); CHECK(g_rs485_jp2.rx_count==1 && g_rs485_link_pass==1);
    CHECK(g_rs485_jp2.rx_timestamp_us==12345);
    make_frame(f,2,1); inject(&jp3,f,48);
    CHECK(g_rs485_jp3.rx_count==1 && g_rs485_link_pass==3);
    make_frame(f,3,2); f[20]^=1; inject(&jp2,f,48);
    CHECK(g_rs485_jp2.bad_frame==1 && g_rs485_jp2.rx_count==1);
    make_frame(f,3,3); inject(&jp2,f,48);
    CHECK(g_rs485_jp2.rx_count==2 && g_rs485_jp2.sequence_errors==1);
    make_frame(f,3,4); inject(&jp2,f,48);
    CHECK(g_rs485_jp2.rx_count==3 && g_rs485_jp2.sequence_errors==1);
    make_frame(f,3,5); inject(&jp2,f,20); ++jp2.epoch; inject(&jp2,f+20,28);
    CHECK(g_rs485_jp2.rx_count==3 && jp2.used==0);
    make_frame(f,3,6); inject(&jp2,f,48);
    CHECK(g_rs485_jp2.rx_count==4);
    return 0;
}
