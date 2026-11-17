#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "reserved_uart2.h"
#include "imu_debug_mode.h"
#include "cascade_left.h"
#include "cascade_right.h"
#include "imu.h"
static CascadeSnapshot left, right;
static ImuData body;
uint32_t imu_time_us(void) { return 100000U; }
void imu_get_body_data(ImuData *out) { *out=body; }
void cascade_left_get_snapshot(CascadeSnapshot *out) { *out=left; }
void cascade_right_get_snapshot(CascadeSnapshot *out) { *out=right; }
static void seed(CascadeSnapshot *s,float base) {
    memset(s,0,sizeof(*s)); s->online=1; s->present_mask=s->valid_mask=s->attitude_mask=7;
    for(unsigned j=0;j<3;j++) {
        s->records[j].age_us=12000; s->records[j].status=2;
        for(unsigned k=0;k<9;k++) s->records[j].values[k]=base+10*j+k;
    }
}
int main(void) {
    td_mcu_frame_t f={0};
    assert(RESERVED_UART2_DATA_VALID_MASK==0x40U && RESERVED_UART2_ATTITUDE_VALID_MASK==0x80U);
    assert(RESERVED_UART2_DEBUG_MODE_VALUE==11U && RESERVED_UART2_IMU_MODE_VALUE==12U);
    assert(reserved_uart2_accept_mode(1)==1);
    assert(reserved_uart2_accept_mode(2)==0 && reserved_uart2_accept_mode(3)==0);
    assert(reserved_uart2_accept_mode(10)==0 && reserved_uart2_accept_mode(19)==0);
    seed(&left,1); seed(&right,101);
    for(unsigned mode=13;mode<=18;mode++) {
        unsigned idx=2-(mode-13)%3;
        const CpRecord *r=mode<16 ? &left.records[idx] : &right.records[idx];
        assert(reserved_uart2_accept_mode(mode)==mode);
        cascade_debug_mode_fill_feedback(&f,mode);
        assert(f.data1&RESERVED_UART2_DATA_VALID_MASK);
        if(idx) {
            assert(fabs(f.data3-r->values[6]*57.2957795f)<0.01f);
            assert(f.data5==r->values[3] && f.data10==r->values[2]);
            assert(f.data1&RESERVED_UART2_ATTITUDE_VALID_MASK);
        } else {
            assert(f.data3==r->values[0] && f.data5==r->values[2]);
            assert(f.data6==2 && fabs(f.data7-12)<0.001f && f.data10==0);
            assert(!(f.data1&RESERVED_UART2_ATTITUDE_VALID_MASK));
        }
    }
    left.attitude_mask=0; cascade_debug_mode_fill_feedback(&f,13);
    assert(f.data3==0 && f.data4==0 && f.data5!=0);
    assert((f.data1&RESERVED_UART2_SENSOR_STATUS_MASK)==RESERVED_UART2_DATA_VALID_MASK);
    left.records[2].age_us=50001; cascade_debug_mode_fill_feedback(&f,13);
    assert(f.data3==0 && f.data5==0 && f.data10==0 && !(f.data1&RESERVED_UART2_SENSOR_STATUS_MASK));
    seed(&left,1); left.online=0; cascade_debug_mode_fill_feedback(&f,15);
    assert(f.data3==0 && f.data7==0 && !(f.data1&RESERVED_UART2_DATA_VALID_MASK));
    seed(&left,1); left.present_mask=0; cascade_debug_mode_fill_feedback(&f,14);
    assert(f.data5==0);
    seed(&left,1); left.valid_mask=0; cascade_debug_mode_fill_feedback(&f,14); assert(f.data5==0);
    body.valid=body.angle_valid=1; body.timestamp_us=99999; body.angle_rad.roll=1;
    body.gyro_rad_s.x=2; body.accel_m_s2.z=9.8f;
    imu_debug_mode_fill_feedback(&f);
    assert(fabs(f.data3-57.2957795f)<0.001f && f.data5==2 && f.data10==9.8f);
    body.timestamp_us=1; imu_debug_mode_fill_feedback(&f);
    assert(f.data3==0 && f.data10==0 && !(f.data1&RESERVED_UART2_SENSOR_STATUS_MASK));
    body.timestamp_us=99999; body.gyro_rad_s.x=NAN; imu_debug_mode_fill_feedback(&f);
    assert(!(f.data1&RESERVED_UART2_DATA_VALID_MASK));
    puts("PASS: mode IDs, six sources, units, pressure, freshness, offline, masks, body IMU and NaN");
    return 0;
}


