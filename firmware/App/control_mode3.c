#include "control_mode3.h"
#include "motor_bus.h"
#include "imu_bus.h"
#include "reserved_uart2.h"
#include "normal_mode.h"
#include <math.h>
#include "reserved_uart1.h"
volatile ControlMode3Monitor g_control_mode3_monitor={CONTROL3_DISABLED,0,0,0,{0,0}};
volatile uint32_t g_control_mode3_torque_enabled;
static uint32_t active, previous;
static uint32_t phase_us;
static uint32_t period_us=2000000U;
volatile float g_control_mode3_amplitude_nm=3.0f, g_control_mode3_period_s=2.0f;
volatile uint32_t g_control_mode3_parameter_rejects;
int control_mode3_set_parameters(float amplitude_nm, float period_s)
{
    if (!isfinite(amplitude_nm) || !isfinite(period_s) || amplitude_nm<0 || amplitude_nm>15 || period_s<0.1f || period_s>60) {
        ++g_control_mode3_parameter_rejects; return 0;
    }
    uint32_t requested_us=(uint32_t)(period_s*1000000.0f+0.5f);
    if (amplitude_nm!=g_control_mode3_amplitude_nm || requested_us!=period_us) {
        control_mode3_reset();
        g_control_mode3_amplitude_nm=amplitude_nm;
        period_us=requested_us;
        g_control_mode3_period_s=(float)period_us/1000000.0f;
    }
    return 1;
}
void control_mode3_reset(void)
{
    if (active) ++g_control_mode3_monitor.resets;
    active=0; phase_us=0;
    g_control_mode3_monitor.elapsed_us=0;
    g_control_mode3_monitor.torque_nm[0]=g_control_mode3_monitor.torque_nm[1]=0;
    g_control_mode3_monitor.status=CONTROL3_DISABLED;
}
void control_mode3_set_torque_enabled(uint32_t enabled)
{
    if (!enabled || !g_control_mode3_torque_enabled) control_mode3_reset();
    g_control_mode3_torque_enabled=enabled ? 1U : 0U;
}
static uint32_t feedback_faults(uint32_t now)
{
    MotorFrame f;
    uint32_t fault=0, age[2]={now-g_motor_left.rx_timestamp_us,now-g_motor_right.rx_timestamp_us};
    if (g_motor_zero_switch || (g_motor_zero_state!=MOTOR_ZERO_IDLE && g_motor_zero_state!=MOTOR_ZERO_DONE)) fault|=CONTROL3_ZEROING;
    if (age[0]>30000U) fault|=CONTROL3_LEFT_OLD;
    if (age[1]>30000U) fault|=CONTROL3_RIGHT_OLD;
    if ((age[0]>age[1] ? age[0]-age[1] : age[1]-age[0])>15000U) fault|=CONTROL3_SKEW;
    for (unsigned i=0;i<2;++i) {
        if (!motor_get_feedback(i,&f)) { fault|=CONTROL3_LEFT_INVALID<<i; continue; }
        if (f.mode!=MOTOR_TORQUE) fault|=CONTROL3_LEFT_MODE<<i;
        if (f.error_flag) fault|=CONTROL3_LEFT_ERROR<<i;
        if (!isfinite(f.ang) || !isfinite(f.speed)) fault|=CONTROL3_LEFT_NUMERIC<<i;
    }
    return fault;
}
static void control_mode3_update(void)
{
    uint32_t now=imu_time_us(), fault;
    if (g_uart2_debug_mode!=RESERVED_UART2_CONTROL_MODE3_VALUE || !g_control_mode3_torque_enabled) { control_mode3_reset(); return; }
    fault=feedback_faults(now);
    /* Feedback faults are diagnostic only in the requested open-loop test. */
    g_control_mode3_monitor.faults|=fault;
    if (fault & CONTROL3_ZEROING) {
        control_mode3_reset(); g_control_mode3_monitor.status=fault; return;
    }
    if (!active) { active=1; previous=now; }
    /* Integrate unsigned microsecond deltas, including timer wrap; bounded phase. */
    phase_us=(phase_us+(uint32_t)(now-previous)%period_us)%period_us;
    previous=now;
    g_control_mode3_monitor.elapsed_us=phase_us;
    g_control_mode3_monitor.torque_nm[0]=g_control_mode3_amplitude_nm*sinf(6.28318530717958647692f*((float)phase_us/(float)period_us));
    /* Left-only test: right torque stays zero, with the same KP as left. */
    g_control_mode3_monitor.torque_nm[1]=0.0f;
    g_control_mode3_monitor.status=fault;
}
int control_mode3_get_command(float nm[2])
{
    nm[0]=nm[1]=0;
    /* Manual stop/mode exit/zeroing still discard phase; feedback does not. */
    control_mode3_update();
    if (!active) return 0;
    nm[0]=g_control_mode3_monitor.torque_nm[0]; nm[1]=g_control_mode3_monitor.torque_nm[1];
    return 1;
}
/* Version 3 uses the same 80-byte UART1 envelope; v1 mode2 is unchanged.
 * Last two floats carry amplitude Nm and period seconds in version 3. */
void control_mode3_poll(void)
{
    static uint32_t publishing, last_publish, sequence;
    uint32_t now=imu_time_us();
    control_mode3_update();
    if (g_uart2_debug_mode!=RESERVED_UART2_CONTROL_MODE3_VALUE) { publishing=0; return; }
    if (publishing && (uint32_t)(now-last_publish)<10000U) return;
    uint32_t elapsed=publishing ? now-last_publish : 0;
    publishing=1; last_publish=now;
    ControlMode2Telemetry t={0};
    t.start=CONTROL2_TELEMETRY_START; t.end=CONTROL2_TELEMETRY_END;
    t.version=3; t.size=sizeof(t); t.sequence=sequence++; t.timestamp_us=now;
    float nm[2];
    if (control_mode3_get_command(nm)) t.flags|=8U;
    t.status=(int32_t)g_control_mode3_monitor.status;
    if (!feedback_faults(now)) {
        MotorFrame f;
        t.flags|=1U;
        for (unsigned i=0;i<2;++i) {
            (void)motor_get_feedback(i,&f);
            t.angle_rad[i]=i ? -f.ang : f.ang;
            t.speed_rad_s[i]=i ? -f.speed : f.speed;
        }
    }
    if (active) {
        t.flags|=2U;
        t.calculated_torque_nm[0]=nm[0]; t.calculated_torque_nm[1]=-nm[1];
    }
    if (g_control_mode3_torque_enabled) t.flags|=4U;
    t.driver_torque_nm[0]=nm[0]; t.driver_torque_nm[1]=nm[1];
    t.period_us=elapsed; t.output_scale=g_control_mode3_amplitude_nm;
    t.average_compute_ms=g_control_mode3_period_s;
    t.compute_us=imu_time_us()-now;
    (void)reserved_uart1_send_control2(&t);
}
void control_mode3_fill_feedback(volatile td_mcu_frame_t *frame)
{
    float nm[2];
    normal_mode_fill_feedback(frame);
    (void)control_mode3_get_command(nm);
    frame->data4=(float)g_control_mode3_monitor.status;
    frame->data5=(float)g_control_mode3_monitor.faults;
    frame->data6=(float)g_control_mode3_monitor.elapsed_us/1000000.0f;
    frame->data7=(float)g_control_mode3_monitor.resets;
    frame->data8=nm[0]; frame->data9=nm[1]; frame->data10=0;
}
