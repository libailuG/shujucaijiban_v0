#include "cascade_receiver.h"
#include <string.h>
#include <math.h>
static int same(const uint32_t a[3], const uint32_t b[3]) { return memcmp(a,b,12)==0; }
void cascade_receiver_init_side(CascadeReceiver *r,uint32_t side) { memset(r,0,sizeof(*r)); r->foot_side_flag=side; }
void cascade_receiver_init(CascadeReceiver *r) { cascade_receiver_init_side(r,0x100U); }
int cascade_receiver_bind(CascadeReceiver *r, const uint32_t uid[3])
{
    if(r->bound) return same(r->root_uid,uid);
    memcpy(r->root_uid,uid,12); r->bound=1; return 1;
}
int cascade_receiver_push(CascadeReceiver *r,const CpPacket *p,uint32_t now,uint32_t extra_age)
{
    uint32_t h[3], mask=0;
    CpRecord records[3]={{0}};
    if(p->type!=CP_PUSH || p->length<12) goto bad;
    memcpy(h,p->payload,12);
    if(h[0]!=CP_THIGH || h[1]>CP_DEGRADED || h[2]<1 || h[2]>3 || p->length!=12+72*h[2]) goto bad;
    for(unsigned i=0;i<h[2];i++) {
        CpRecord a; memcpy(&a,p->payload+12+72*i,72);
        if(a.role<1 || a.role>3 || (mask&(1U<<(a.role-1)))) goto bad;
        unsigned j=a.role-1;
        for(unsigned k=0;k<9;k++) if(!isfinite(a.values[k])) goto bad;
        if(a.role==CP_FOOT) {
            if((r->foot_side_flag!=0x100U && r->foot_side_flag!=0x200U) ||
               (a.flags&0x300U)!=r->foot_side_flag || a.status<0 || a.status>3) goto bad;
            /* 3.3V ADC tolerance. These are volts, not force. */
            for(unsigned k=0;k<3;k++) if(a.values[k]<0 || a.values[k]>3.6f) goto bad;
        } else if(a.status>0) goto bad;
        if((r->source_mask&(1U<<j)) && !same(r->source_uid[j],a.uid)) goto bad;
        records[j]=a; mask|=1U<<j;
    }
    if(!(mask&4U)) goto bad;
    if(r->bound && !same(r->root_uid,records[2].uid)) goto bad;
    if(!r->bound) {
        if(!r->candidate_count || !same(r->candidate_uid,records[2].uid)) {
            memcpy(r->candidate_uid,records[2].uid,12); r->candidate_count=1;
            r->candidate_mask=0;
        } else {
            int consistent=1;
            for(unsigned j=0;j<3;j++) if((mask&r->candidate_mask&(1U<<j)) &&
                !same(r->candidate_source_uid[j],records[j].uid)) consistent=0;
            if(!consistent) { r->candidate_count=1; r->candidate_mask=0; }
            else if(p->sequence!=r->candidate_sequence) ++r->candidate_count;
        }
        for(unsigned j=0;j<3;j++) if(mask&(1U<<j)) memcpy(r->candidate_source_uid[j],records[j].uid,12);
        r->candidate_mask|=mask;
        r->candidate_sequence=p->sequence;
        if(r->candidate_count<3) return 0;
        cascade_receiver_bind(r,records[2].uid);
    }
    if(r->accepted) {
        uint32_t delta=p->sequence-r->data.push_sequence;
        if(!delta) { ++r->duplicate_push; return 0; }
        if(delta>=0x80000000U && (uint32_t)(now-r->data.received_us)<100000U) goto bad;
        if(delta<0x80000000U) r->skipped_push+=delta-1;
    }
    for(unsigned j=0;j<3;j++) if(mask&(1U<<j)) {
        if(!(r->source_mask&(1U<<j))) memcpy(r->source_uid[j],records[j].uid,12);
        if(!(r->sequence_mask&(1U<<j)) || r->previous_sequence[j]!=records[j].sequence) ++r->new_samples[j];
        r->previous_sequence[j]=records[j].sequence;
        cp_age_record(&records[j],extra_age);
    }
    r->source_mask|=mask; r->sequence_mask|=mask;
    memcpy(r->data.records,records,sizeof(records));
    r->data.present_mask=mask; r->data.topology=h[1]; r->data.received_us=now;
    r->data.push_sequence=p->sequence; r->data.online=1; ++r->accepted;
    return 1;
bad:
    r->candidate_count=0; ++r->rejected; return 0;
}
void cascade_receiver_snapshot(const CascadeReceiver *r,uint32_t now,CascadeSnapshot *out)
{
    *out=r->data; out->valid_mask=out->attitude_mask=0;
    uint32_t elapsed=now-out->received_us;
    if(!r->accepted || elapsed>=100000U) out->online=0;
    for(unsigned j=0;j<3;j++) {
        CpRecord *a=&out->records[j]; cp_age_record(a,elapsed);
        if(!out->online) a->flags&=~(CP_VALID|CP_ATTITUDE_VALID);
        if(!(out->present_mask&(1U<<j))) continue;
        if((a->flags&CP_VALID) && (j==0 || a->status==0)) {
            out->valid_mask|=1U<<j;
            if(j && (a->flags&CP_ATTITUDE_VALID)) out->attitude_mask|=1U<<j;
        }
    }
}
