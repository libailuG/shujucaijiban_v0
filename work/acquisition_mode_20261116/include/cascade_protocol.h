#ifndef CASCADE_PROTOCOL_H
#define CASCADE_PROTOCOL_H
#include <stdint.h>
#include <stddef.h>
#define CP_VERSION 1U
#define CP_BAUD 2000000U
#define CP_MAX_PAYLOAD 240U
#define CP_MAX_FRAME (CP_MAX_PAYLOAD + 14U)
#define CP_INFO 1U
#define CP_DATA 2U
#define CP_STATUS 3U
#define CP_REDISCOVER 4U
#define CP_CONFIG 5U
#define CP_STREAM 6U
#define CP_PUSH 0x90U
#define CP_CAP_STREAM 8U
#define CP_POLL_US 2000U
#define CP_REPLY 0x80U
#define CP_UNKNOWN 0U
#define CP_FOOT 1U
#define CP_SHANK 2U
#define CP_THIGH 3U
#define CP_WAITING 0U
#define CP_ONLINE 1U
#define CP_OFFLINE 2U
#define CP_MISMATCH 3U
#define CP_DEGRADED 4U
#define CP_VALID 1U
#define CP_ATTITUDE_VALID 2U
#define CP_FRESH_US 50000U
#ifndef CP_REPLY_DELAY_US
#define CP_REPLY_DELAY_US 250U
#endif

typedef struct {
    uint32_t role, uid[3], sequence, sample_us, age_us, flags;
    int32_t status; /* Foot: contact 0..3. IMU: driver result. */
    float values[9]; /* Foot: first 3 = V; IMU: m/s2, rad/s, rad. */
} CpRecord;
typedef char CpRecordLayout[(sizeof(float)==4 && sizeof(CpRecord)==72 && offsetof(CpRecord,values)==36)?1:-1];
typedef struct {
    uint32_t role, uid[3], capabilities, sample_period_us;
    uint32_t downstream_role, topology, poll_period_us, protocol_errors;
} CpInfo;
typedef char CpInfoLayout[(sizeof(CpInfo)==40)?1:-1];
typedef struct { uint8_t type; uint16_t length; uint32_t sequence; uint8_t payload[CP_MAX_PAYLOAD]; } CpPacket;
typedef struct {
    uint8_t bytes[CP_MAX_FRAME]; uint16_t used;
    uint32_t bad_crc, bad_length, discarded;
} CpParser;
uint16_t cp_crc(const uint8_t *p, uint16_t length);
uint16_t cp_encode(uint8_t *out, uint8_t type, uint32_t seq, const void *payload, uint16_t length);
int cp_feed(CpParser *parser, uint8_t byte, CpPacket *out);
void cp_parser_reset(CpParser *parser);
uint32_t cp_age(uint32_t original, uint32_t elapsed);
void cp_age_record(CpRecord *record, uint32_t elapsed);
uint32_t cp_role_for_child(uint32_t child);
#endif

