#include "App/motor_protocol.h"
#define CHECK(x) do { if(!(x)) return __LINE__; } while(0)
int test_main(void) {
    uint8_t wire[49]={0}, feedback[48];
    MotorFrame f={0},r;
    f.motor_id=126; motor_encode(wire+1,&f);
    for(unsigned i=0;i<48;i++) {
        uint8_t expected=i<4?0xAA:(i==4?126:(i>=44?0xEE:0));
        CHECK(wire[i+1]==expected);
    }
    /* Every payload word distinct, including nonzero error_flag at offset40. */
    for(unsigned i=0;i<12;i++) motor_put(feedback+4*i,0x3F800000U+i);
    motor_put(feedback,0xBBBBBBBBU); motor_put(feedback+44,0xDDDDDDDDU);
    motor_put(feedback+4,127); motor_put(feedback+8,1004);
    CHECK(motor_decode(&r,feedback)); CHECK(r.motor_id==127 && r.mode==1004);
    CHECK(r.error_flag==0x3F80000AU);
    motor_encode(wire+1,&r);
    for(unsigned i=4;i<44;i++) CHECK(wire[i+1]==feedback[i]);
    CHECK(!motor_decode(&r,wire+1)); /* Reject outgoing direction. */
    MotorStream s={0}; unsigned count=0;
    for(unsigned split=0;split<=48;split++) {
        s.used=0;
        for(unsigned i=0;i<split;i++) count+=motor_stream_push(&s,feedback[i],&r);
        for(unsigned i=split;i<48;i++) count+=motor_stream_push(&s,feedback[i],&r);
        CHECK(s.used==0);
    }
    CHECK(count==49);
    for(unsigned i=0;i<20;i++) CHECK(!motor_stream_push(&s,0xBB,&r));
    for(unsigned i=0;i<48;i++) count+=motor_stream_push(&s,feedback[i],&r);
    CHECK(count==50);
    feedback[44]=0;
    for(unsigned i=0;i<48;i++) CHECK(!motor_stream_push(&s,feedback[i],&r));
    feedback[44]=0xDD;
    for(unsigned k=0;k<2;k++) for(unsigned i=0;i<48;i++) count+=motor_stream_push(&s,feedback[i],&r);
    CHECK(count==52 && s.bad_tail>0);
    return 0;
}
