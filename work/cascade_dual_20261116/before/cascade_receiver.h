#ifndef CASCADE_RECEIVER_H
#define CASCADE_RECEIVER_H
#include "cascade_protocol.h"
typedef struct {
    CpRecord records[3]; /* role-1: left foot, shank, thigh. */
    uint32_t present_mask, valid_mask, attitude_mask;
    uint32_t topology, received_us, push_sequence, online;
} CascadeSnapshot;
typedef struct {
    CascadeSnapshot data;
    uint32_t root_uid[3], source_uid[3][3], candidate_uid[3];
    uint32_t candidate_source_uid[3][3], candidate_mask;
    uint32_t bound, source_mask, candidate_count, candidate_sequence;
    uint32_t accepted, rejected, duplicate_push, skipped_push;
    uint32_t new_samples[3], previous_sequence[3], sequence_mask;
} CascadeReceiver;
void cascade_receiver_init(CascadeReceiver *r);
int cascade_receiver_bind(CascadeReceiver *r, const uint32_t uid[3]);
int cascade_receiver_push(CascadeReceiver *r, const CpPacket *p, uint32_t now, uint32_t extra_age);
void cascade_receiver_snapshot(const CascadeReceiver *r, uint32_t now, CascadeSnapshot *out);
#endif
