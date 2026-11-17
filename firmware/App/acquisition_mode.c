#include "acquisition_mode.h"
#include "reserved_uart2.h"
#include "reserved_uart1.h"
#include "motor_bus.h"
#include "imu.h"
#include "imu_bus.h"
#include "cascade_left.h"
#include "cascade_right.h"
#include <math.h>
#include <string.h>
static uint32_t enabled, started, previous_send;
void acquisition_mode_set_enabled(uint32_t value)
{
    value=value ? 1U : 0U;
    if (value!=enabled) started=0U;
    enabled=value;
}
uint32_t acquisition_mode_is_enabled(void) { return enabled; }
static int limb(const CascadeSnapshot *s,unsigned index,float sign,float *angle,float *speed)
{
    const CpRecord *r=&s->records[index];
    uint32_t mask=1U<<index;
    if (!s->online || !(s->present_mask&mask) || !(s->valid_mask&mask) ||
        r->age_us>CP_FRESH_US || !isfinite(r->values[3])) return 0;
    *speed=sign*r->values[3];
    if (!(s->attitude_mask&mask) || !isfinite(r->values[6])) return 0;
    /* Correct direction first, then subtract the pi/2 neutral angle. */
    *angle=sign*r->values[6]-1.5707963267948966f;
    return 1;
}
static int pressure(const CascadeSnapshot *s,uint32_t *value)
{
    const CpRecord *r=&s->records[0];
    if (!s->online || !(s->present_mask&1U) || !(s->valid_mask&1U) ||
        r->age_us>CP_FRESH_US || r->status<0 || r->status>3) return 0;
    *value=(uint32_t)r->status;
    return 1;
}
uint32_t acquisition_mode_snapshot(AcquisitionFrame *f)
{
    uint32_t now=imu_time_us();
    MotorFrame m;
    CascadeSnapshot left,right;
    memset(f,0,sizeof(*f));
    f->start_flag=TD_MCU_START; f->timestamp_us=now; f->end_flag=TD_MCU_END;
    int valid=1;
    if (motor_get_feedback(0,&m) && !m.error_flag && g_motor_left.rx_valid &&
        (uint32_t)(now-g_motor_left.rx_timestamp_us)<=30000U && isfinite(m.ang) && isfinite(m.speed)) {
        f->left_motor_angle=m.ang; f->left_motor_speed=m.speed;
    } else valid=0;
    if (motor_get_feedback(1,&m) && !m.error_flag && g_motor_right.rx_valid &&
        (uint32_t)(now-g_motor_right.rx_timestamp_us)<=30000U && isfinite(m.ang) && isfinite(m.speed)) {
        f->right_motor_angle=-m.ang; f->right_motor_speed=-m.speed;
    } else valid=0;
    ImuData body; imu_get_body_data(&body);
    if (body.valid && !body.last_error && (uint32_t)(now-body.timestamp_us)<=CP_FRESH_US &&
        isfinite(body.gyro_rad_s.x) && isfinite(body.gyro_rad_s.y) && isfinite(body.accel_m_s2.z)) {
        f->body_gyro_x=body.gyro_rad_s.x; f->body_gyro_y=body.gyro_rad_s.y;
        f->body_accel_z=body.accel_m_s2.z;
    }
    cascade_left_get_snapshot(&left); cascade_right_get_snapshot(&right);
    int attitude=limb(&left,2,1,&f->left_thigh_angle,&f->left_thigh_speed);
    attitude &= limb(&left,1,1,&f->left_shank_angle,&f->left_shank_speed);
    attitude &= limb(&right,2,-1,&f->right_thigh_angle,&f->right_thigh_speed);
    attitude &= limb(&right,1,-1,&f->right_shank_angle,&f->right_shank_speed);
    valid &= pressure(&left,&f->left_pressure);
    valid &= pressure(&right,&f->right_pressure);
    return (valid && attitude ? RESERVED_UART2_DATA_VALID_MASK : 0U) |
        (attitude ? RESERVED_UART2_ATTITUDE_VALID_MASK : 0U);
}
void acquisition_mode_encode(uint8_t out[ACQUISITION_FRAME_SIZE],const AcquisitionFrame *f)
{
    uint32_t words[20]; memcpy(words,f,sizeof(words));
    words[0]=TD_MCU_START; words[19]=TD_MCU_END;
    for(unsigned i=0;i<20;i++) td_mcu_put_u32(out+4*i,words[i]);
}
void acquisition_mode_fill_feedback(volatile td_mcu_frame_t *f)
{
    AcquisitionFrame a;
    uint32_t status=acquisition_mode_snapshot(&a);
    f->data1=(f->data1 & ~RESERVED_UART2_SENSOR_STATUS_MASK)|status;
    /* UART2 display uses degrees; UART1 acquisition retains radians. */
    const float rad_to_deg=57.29577951308232f;
    f->data3=a.left_motor_angle*rad_to_deg; f->data4=a.right_motor_angle*rad_to_deg;
    f->data5=a.left_thigh_angle*rad_to_deg; f->data6=a.left_shank_angle*rad_to_deg;
    f->data7=(float)a.left_pressure;
    f->data8=a.right_thigh_angle*rad_to_deg; f->data9=a.right_shank_angle*rad_to_deg;
    f->data10=(float)a.right_pressure;
}
void acquisition_mode_poll(void)
{
    if (g_uart2_debug_mode!=RESERVED_UART2_ACQUISITION_VALUE || !enabled) { started=0; return; }
    uint32_t now=imu_time_us();
    if (started && (uint32_t)(now-previous_send)<ACQUISITION_PERIOD_US) return;
    started=1; previous_send=now;
    AcquisitionFrame frame; uint8_t bytes[ACQUISITION_FRAME_SIZE];
    (void)acquisition_mode_snapshot(&frame);
    acquisition_mode_encode(bytes,&frame);
    (void)reserved_uart1_send_acquisition(bytes);
}





