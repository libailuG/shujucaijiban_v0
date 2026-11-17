#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "cascade_receiver.h"
#define CHECK(x) do { if(!(x)) { printf("FAIL line %d: %s\n",__LINE__,#x); exit(1); } ++checks; } while(0)
static unsigned checks;
static CascadeReceiver r;
static CpParser parser;
static CpPacket p,q;
static uint8_t wire[CP_MAX_FRAME];
static void make(uint32_t seq)
{
    uint32_t h[3]={3,1,3}; CpRecord a[3]={{0}};
    for(unsigned j=0;j<3;j++) {
        a[j].role=j+1; a[j].uid[0]=j+100; a[j].sequence=seq;
        a[j].flags=j?3:0x101; a[j].age_us=4000;
    }
    a[0].values[0]=2.6f; a[0].status=2;
    p.type=CP_PUSH; p.sequence=seq; p.length=228;
    memcpy(p.payload,h,12); memcpy(p.payload+12,a,sizeof(a));
}
static CpRecord get(unsigned j) { CpRecord a; memcpy(&a,p.payload+12+72*j,72); return a; }
static void put(unsigned j,CpRecord a) { memcpy(p.payload+12+72*j,&a,72); }
int main(void)
{
    CHECK(cp_crc((const uint8_t *)"123456789",9)==0x29b1);
    CHECK(cp_encode(wire,CP_INFO,1,0,0)==14 && wire[12]==0x7f && wire[13]==0x4b);
    uint32_t period=2000;
    CHECK(cp_encode(wire,CP_STREAM,2,&period,4)==18 && wire[16]==0xd4 && wire[17]==0xe5);
    make(1); CHECK(cp_encode(wire,p.type,p.sequence,p.payload,p.length)==242);
    for(unsigned i=0;i<242;i++) CHECK(cp_feed(&parser,wire[i],&q)==(i==241));
    CHECK(q.length==228 && memcmp(q.payload,p.payload,228)==0);
    wire[60]^=1; unsigned completed=0;
    for(unsigned i=0;i<242;i++) completed+=cp_feed(&parser,wire[i],&q);
    CHECK(!completed && parser.bad_crc>0);
    cp_encode(wire,p.type,p.sequence,p.payload,p.length);
    for(unsigned i=0;i<242;i++) completed+=cp_feed(&parser,wire[i],&q);
    CHECK(completed==1);
    cascade_receiver_init(&r);
    CHECK(!cascade_receiver_push(&r,&p,1000,1210));
    CHECK(!cascade_receiver_push(&r,&p,2000,1210)); /* duplicate cannot bind */
    p.sequence=2; CHECK(!cascade_receiver_push(&r,&p,3000,1210));
    p.sequence=3; CHECK(cascade_receiver_push(&r,&p,4000,1210));
    CHECK(r.bound && r.accepted==1);
    CascadeSnapshot s; cascade_receiver_snapshot(&r,5000,&s);
    CHECK(s.online && s.valid_mask==7 && s.attitude_mask==6 && s.records[0].age_us==6210);
    CHECK(!cascade_receiver_push(&r,&p,6000,1210) && r.duplicate_push==1);
    make(4); CpRecord a=get(0); a.flags=0x201; put(0,a);
    CHECK(!cascade_receiver_push(&r,&p,7000,1210)); /* wrong side */
    make(4); a=get(1); a.uid[0]++; put(1,a);
    CHECK(!cascade_receiver_push(&r,&p,7000,1210)); /* replacement source */
    make(4); a=get(1); a.role=1; put(1,a);
    CHECK(!cascade_receiver_push(&r,&p,7000,1210)); /* duplicate role */
    make(4); uint32_t nan_bits=0x7fc00000; a=get(1); memcpy(&a.values[0],&nan_bits,4); put(1,a);
    CHECK(!cascade_receiver_push(&r,&p,7000,1210));
    make(4); p.length--; CHECK(!cascade_receiver_push(&r,&p,7000,1210));
    make(4); a=get(2); a.age_us=49000; put(2,a);
    CHECK(cascade_receiver_push(&r,&p,8000,1210));
    cascade_receiver_snapshot(&r,8000,&s); CHECK(s.valid_mask==3);
    make(5); a=get(2); memcpy(p.payload+12,&a,72); p.length=84; uint32_t one=1; memcpy(p.payload+8,&one,4);
    CHECK(cascade_receiver_push(&r,&p,10000,1210));
    cascade_receiver_snapshot(&r,10000,&s); CHECK(s.present_mask==4 && s.valid_mask==4 && s.records[0].role==0);
    cascade_receiver_snapshot(&r,110000,&s); CHECK(!s.online && !s.valid_mask);
    CHECK(cp_age(UINT32_MAX-3,10)==UINT32_MAX);
    make(0xffffffffU); cascade_receiver_init(&r); CHECK(cascade_receiver_bind(&r,get(2).uid));
    CHECK(cascade_receiver_push(&r,&p,0xfffffff0U,1210));
    make(0); CHECK(cascade_receiver_push(&r,&p,0x10U,1210));
    cascade_receiver_snapshot(&r,0x20U,&s); CHECK(s.online && s.valid_mask==7);
    /* Two receivers cannot accept each other's pressure-side marker. */
    CascadeReceiver right;
    cascade_receiver_init_side(&right,0x200U); make(1);
    CHECK(!cascade_receiver_push(&right,&p,1000,1210));
    a=get(0); a.flags=0x201; put(0,a);
    CHECK(!cascade_receiver_push(&right,&p,2000,1210));
    p.sequence=2; CHECK(!cascade_receiver_push(&right,&p,4000,1210));
    p.sequence=3; CHECK(cascade_receiver_push(&right,&p,6000,1210));
    cascade_receiver_snapshot(&right,6000,&s); CHECK(s.valid_mask==7 && s.records[0].flags==0x201);
    CHECK(!cascade_receiver_push(&r,&p,6000,1210));
    CHECK(r.data.records[0].flags==0x101 && right.data.records[0].flags==0x201);
    cascade_receiver_snapshot(&right,106000,&s); CHECK(!s.online && !s.valid_mask);
    p.sequence=4; CHECK(cascade_receiver_push(&right,&p,108000,1210));
    cascade_receiver_snapshot(&right,108000,&s); CHECK(s.online && s.valid_mask==7);
    CHECK(r.data.push_sequence==0); /* Other link remains independent. */
    printf("PASS %u checks: framing, CRC recovery, binding, left/right isolation, freshness, rollover\n",checks);
    return 0;
}
