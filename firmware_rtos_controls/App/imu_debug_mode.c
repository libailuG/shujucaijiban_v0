#include "imu_debug_mode.h"
#include "imu.h"
#include "imu_bus.h"
#include "reserved_uart2.h"
#include "cascade_left.h"
#include "cascade_right.h"
#include <math.h>

static void clear_feedback(volatile td_mcu_frame_t *f)
{
    f->data1 &= ~RESERVED_UART2_SENSOR_STATUS_MASK;
    f->data3=f->data4=f->data5=f->data6=0.0f;
    f->data7=f->data8=f->data9=f->data10=0.0f;
}

/* Shared layout: Roll/Pitch degrees, XYZ gyro rad/s, XYZ acceleration m/s2.
 * Body uses its mounting correction; cascade boards retain their own axes. */
static void fill_imu(volatile td_mcu_frame_t *f, const float a[3],
                     const float g[3], const float angles[2], uint32_t attitude)
{
    for (unsigned i=0;i<3;i++) if (!isfinite(a[i]) || !isfinite(g[i])) return;
    f->data1 |= RESERVED_UART2_DATA_VALID_MASK;
    f->data5=g[0]; f->data6=g[1]; f->data7=g[2];
    f->data8=a[0]; f->data9=a[1]; f->data10=a[2];
    if (attitude && isfinite(angles[0]) && isfinite(angles[1])) {
        f->data1 |= RESERVED_UART2_ATTITUDE_VALID_MASK;
        f->data3=angles[0]*57.29577951308232f;
        f->data4=angles[1]*57.29577951308232f;
    }
}

void imu_debug_mode_fill_feedback(volatile td_mcu_frame_t *f)
{
    ImuData imu;
    clear_feedback(f);
    imu_get_body_data(&imu);
    if (!imu.valid || imu.last_error ||
        (uint32_t)(imu_time_us()-imu.timestamp_us)>CP_FRESH_US) return;
    const float a[3]={imu.accel_m_s2.x,imu.accel_m_s2.y,imu.accel_m_s2.z};
    const float g[3]={imu.gyro_rad_s.x,imu.gyro_rad_s.y,imu.gyro_rad_s.z};
    const float angles[2]={imu.angle_rad.roll,imu.angle_rad.pitch};
    fill_imu(f,a,g,angles,imu.angle_valid);
}

void cascade_debug_mode_fill_feedback(volatile td_mcu_frame_t *f, uint32_t mode)
{
    CascadeSnapshot snapshot;
    clear_feedback(f);
    if (mode<RESERVED_UART2_LEFT_THIGH_VALUE || mode>RESERVED_UART2_RIGHT_PRESSURE_VALUE) return;
    if (mode>=RESERVED_UART2_RIGHT_THIGH_VALUE) cascade_right_get_snapshot(&snapshot);
    else cascade_left_get_snapshot(&snapshot);
    unsigned offset=(mode-RESERVED_UART2_LEFT_THIGH_VALUE)%3U;
    unsigned index=2U-offset; /* thigh, shank, foot -> records[2,1,0] */
    const CpRecord *r=&snapshot.records[index];
    uint32_t mask=1U<<index;
    if (!snapshot.online || !(snapshot.present_mask&mask) || !(snapshot.valid_mask&mask) ||
        r->age_us>CP_FRESH_US) return;
    if (index==0U) {
        for (unsigned i=0;i<3;i++) if (!isfinite(r->values[i])) return;
        f->data1 |= RESERVED_UART2_DATA_VALID_MASK;
        f->data3=r->values[0]; f->data4=r->values[1]; f->data5=r->values[2];
        f->data6=(float)r->status;
        f->data7=(float)r->age_us*0.001f;
    } else {
        fill_imu(f,&r->values[0],&r->values[3],&r->values[6],snapshot.attitude_mask&mask);
    }
}
