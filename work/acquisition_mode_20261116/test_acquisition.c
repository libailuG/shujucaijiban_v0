#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "acquisition_mode.h"
#include "reserved_uart2.h"
#include "motor_bus.h"
#include "imu.h"
#include "cascade_left.h"
#include "cascade_right.h"
volatile uint32_t g_uart2_debug_mode;
volatile MotorData g_motor_left,g_motor_right;
static MotorFrame motors[2];
static CascadeSnapshot left,right;
static ImuData body;
static uint32_t now=100000,sends;
static uint8_t sent[80];
uint32_t imu_time_us(void){return now;}
int motor_get_feedback(uint32_t side,MotorFrame *out){*out=motors[side]; return 1;}
void imu_get_body_data(ImuData *out){*out=body;}
void cascade_left_get_snapshot(CascadeSnapshot *out){*out=left;}
void cascade_right_get_snapshot(CascadeSnapshot *out){*out=right;}
int reserved_uart1_send_acquisition(const uint8_t *frame){memcpy(sent,frame,80); ++sends; return 1;}
static void seed(CascadeSnapshot *s,float base) {
    memset(s,0,sizeof(*s)); s->online=1; s->present_mask=s->valid_mask=s->attitude_mask=7;
    s->records[0].status=(int)base;
    for(unsigned j=1;j<3;j++){s->records[j].values[6]=base+j; s->records[j].values[3]=base+j+10;}
}
int main(void){
    AcquisitionFrame a; td_mcu_frame_t f={0}; uint8_t bytes[80];
    assert(reserved_uart2_accept_mode(20)==20);
    assert(ACQUISITION_PERIOD_US==10000 && sizeof(a)==80);
    motors[0].ang=1; motors[0].speed=2; motors[1].ang=3; motors[1].speed=4;
    g_motor_left.rx_valid=g_motor_right.rx_valid=1;
    g_motor_left.rx_timestamp_us=g_motor_right.rx_timestamp_us=now;
    body.valid=1; body.timestamp_us=now; body.gyro_rad_s.x=5; body.gyro_rad_s.y=6; body.accel_m_s2.z=7;
    seed(&left,1); seed(&right,2);
    assert(acquisition_mode_snapshot(&a)==0xc0);
    assert(a.timestamp_us==now);
    assert(a.start_flag==TD_MCU_START && a.end_flag==TD_MCU_END);
    assert(a.left_motor_angle==1 && a.left_motor_speed==2 && a.right_motor_angle==-3 && a.right_motor_speed==-4);
    assert(a.body_gyro_x==5 && a.body_gyro_y==6 && a.body_accel_z==7);
    assert(fabs(a.left_thigh_angle-(3.0f-1.5707963267948966f))<0.00001f && a.left_thigh_speed==13 && fabs(a.left_shank_angle-(2.0f-1.5707963267948966f))<0.00001f && a.left_shank_speed==12);
    assert(fabs(a.right_thigh_angle-(-4.0f-1.5707963267948966f))<0.00001f && a.right_thigh_speed==-14 && fabs(a.right_shank_angle-(-3.0f-1.5707963267948966f))<0.00001f && a.right_shank_speed==-13);
    assert(a.left_pressure==1 && a.right_pressure==2);
    acquisition_mode_encode(bytes,&a);
    assert(bytes[0]==0xaa && bytes[79]==0xee && bytes[52]==1 && bytes[53]==0 && bytes[72]==2);
    assert(td_mcu_get_float(bytes+20)==-4 && fabs(td_mcu_get_float(bytes+36)-(3.0f-1.5707963267948966f))<0.00001f);
    assert(td_mcu_get_float(bytes+40)==13 && fabs(td_mcu_get_float(bytes+44)-(2.0f-1.5707963267948966f))<0.00001f && td_mcu_get_float(bytes+48)==12);
    assert(fabs(td_mcu_get_float(bytes+56)-(-4.0f-1.5707963267948966f))<0.00001f && td_mcu_get_float(bytes+60)==-14);
    assert(fabs(td_mcu_get_float(bytes+64)-(-3.0f-1.5707963267948966f))<0.00001f && td_mcu_get_float(bytes+68)==-13);
    assert(motor_u32(bytes+4)==a.timestamp_us);
    acquisition_mode_fill_feedback(&f);
    assert(fabs(f.data3-57.2957795f)<0.001f && fabs(f.data4+171.8873385f)<0.001f);
    assert(fabs(f.data5-(171.8873385f-90.0f))<0.001f && fabs(f.data6-(114.591559f-90.0f))<0.001f && f.data7==1);
    assert(fabs(f.data8+(229.183118f+90.0f))<0.001f && fabs(f.data9+(171.8873385f+90.0f))<0.001f && f.data10==2 && (f.data1&0xc0)==0xc0);
    g_uart2_debug_mode=20;
    acquisition_mode_poll(); assert(sends==0);
    acquisition_mode_set_enabled(1); acquisition_mode_poll(); assert(sends==1);
    now+=9999; acquisition_mode_poll(); assert(sends==1);
    now++; acquisition_mode_poll(); assert(sends==2);
    acquisition_mode_set_enabled(0); now+=10000; acquisition_mode_poll(); assert(sends==2);
    acquisition_mode_set_enabled(1); acquisition_mode_poll(); assert(sends==3);
    g_uart2_debug_mode=0; now+=10000; acquisition_mode_poll(); assert(sends==3);
    right.online=0; assert(!(acquisition_mode_snapshot(&a)&0xc0));
    assert(a.right_thigh_angle==0 && a.right_pressure==0 && fabs(a.left_thigh_angle-(3.0f-1.5707963267948966f))<0.00001f);
    seed(&right,2); left.attitude_mask=0;
    assert(!(acquisition_mode_snapshot(&a)&0xc0) && a.left_thigh_angle==0 && a.left_thigh_speed==13);
    left.online=0; acquisition_mode_fill_feedback(&f); assert(f.data5==0 && f.data7==0 && !(f.data1&0xc0));
    now+=50001; acquisition_mode_snapshot(&a);
    assert(a.left_motor_angle==0 && a.body_gyro_x==0);
    /* Neutral is +90 degrees after direction correction: raw left +pi/2,
     * raw right -pi/2. Check both wire formats and +/- deviations. */
    seed(&left,1); seed(&right,2);
    for(unsigned j=1;j<3;j++) {
        left.records[j].values[6]=1.5707963267948966f;
        right.records[j].values[6]=-1.5707963267948966f;
    }
    acquisition_mode_snapshot(&a);
    assert(a.left_thigh_angle==0 && a.left_shank_angle==0 && a.right_thigh_angle==0 && a.right_shank_angle==0);
    acquisition_mode_encode(bytes,&a);
    assert(td_mcu_get_float(bytes+36)==0 && td_mcu_get_float(bytes+44)==0);
    assert(td_mcu_get_float(bytes+56)==0 && td_mcu_get_float(bytes+64)==0);
    assert(motor_u32(bytes+4)==a.timestamp_us);
    acquisition_mode_fill_feedback(&f);
    assert(f.data5==0 && f.data6==0 && f.data8==0 && f.data9==0);
    left.records[2].values[6]+=0.25f;
    right.records[2].values[6]+=0.25f;
    acquisition_mode_snapshot(&a);
    assert(fabs(a.left_thigh_angle-0.25f)<0.00001 && fabs(a.right_thigh_angle+0.25f)<0.00001);
    assert(motor_u32(bytes+4)==a.timestamp_us);
    acquisition_mode_fill_feedback(&f);
    assert(fabs(f.data5-14.3239449f)<0.001 && fabs(f.data8+14.3239449f)<0.001);
    puts("PASS: acquisition mapping, signs, rad units, uint32 flags, 80-byte LE encoding, device timestamp, 100Hz, switch gating, mode exit, invalid data");
    return 0;
}




