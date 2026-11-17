#include "control_mode2.h"
#include "control2_policy/g1_assist_v10_policy.h"
#include "control2_policy/assist_oscillation_monitor.h"
#include "motor_bus.h"
#include "imu_bus.h"
#include "reserved_uart2.h"
#include "normal_mode.h"
#include "reserved_uart1.h"
#include <math.h>

#define PERIOD_US 10000U
#define MAX_PERIOD_US 11000U
#define TORQUE_LIMIT_NM 15.0f
static const float output_scales[8]={0.6f,0.714285714f,0.828571429f,0.942857143f,
    1.057142857f,1.171428571f,1.285714286f,1.4f};
volatile uint32_t g_control_mode2_output_level=UINT32_MAX;
volatile float g_control_mode2_output_scale=0.0f;
static G1AssistV10Controller controller;
static AssistOscillationMonitor_t oscillation;
static uint32_t active, previous_tick;
static uint64_t period_sum_us, period_samples;
static uint64_t compute_sum_us, compute_samples;
static float command[2];
static float calculated_torque[2];
/* Temporary diagnostic mode: retain latest finite feedback, ignoring its
 * age, inter-motor skew, protocol mode and motor error flag. */
static MotorFrame retained_feedback[2];
volatile uint32_t g_control_mode2_torque_enabled;
volatile ControlMode2Monitor g_control_mode2_monitor = { .status=CONTROL2_DISABLED };

void control_mode2_set_output_scale(float scale)
{
    if (!isfinite(scale) || scale<0.0f) scale=0.0f;
    if (scale!=g_control_mode2_output_scale) {
        /* Never transmit the old-scale command while awaiting the next tick.
         * Unscaled policy history is preserved across coefficient updates. */
        command[0]=command[1]=0.0f;
        calculated_torque[0]=calculated_torque[1]=0.0f;
        g_control_mode2_monitor.driver_torque_nm[0]=0.0f;
        g_control_mode2_monitor.driver_torque_nm[1]=0.0f;
    }
    g_control_mode2_output_scale=scale;
    g_control_mode2_output_level=UINT32_MAX;
}

int control_mode2_set_output_level(uint32_t level)
{
    if (level>=8U) return 0;
    control_mode2_set_output_scale(output_scales[level]);
    g_control_mode2_output_level=level;
    return 1;
}

void control_mode2_set_torque_enabled(uint32_t enabled)
{
    g_control_mode2_torque_enabled=enabled ? 1U : 0U;
}

void control_mode2_reset(void)
{
    if (active) ++g_control_mode2_monitor.resets;
    g1_assist_v10_stop(&controller,0);
    AssistOscillationMonitor_Reset(&oscillation);
    active=0U;
    period_sum_us=period_samples=0;
    g_control_mode2_monitor.average_period_ms=0;
    compute_sum_us=compute_samples=0;
    g_control_mode2_monitor.average_compute_ms=0;
    command[0]=command[1]=0.0f;
    calculated_torque[0]=calculated_torque[1]=0.0f;
    g_control_mode2_monitor.model_torque_nm[0]=g_control_mode2_monitor.model_torque_nm[1]=0;
    g_control_mode2_monitor.driver_torque_nm[0]=g_control_mode2_monitor.driver_torque_nm[1]=0;
    g_control_mode2_monitor.guard_latched=g_control_mode2_monitor.oscillation_latched=0U;
    g_control_mode2_monitor.status=CONTROL2_DISABLED;
}

static int read_feedback(uint32_t now, MotorFrame frames[2])
{
    (void)now;
    if (g_motor_zero_switch || (g_motor_zero_state!=MOTOR_ZERO_IDLE &&
        g_motor_zero_state!=MOTOR_ZERO_DONE)) return 0;
    for (unsigned i=0;i<2;++i) {
        MotorFrame candidate;
        if (motor_get_feedback(i,&candidate) && isfinite(candidate.ang) && isfinite(candidate.speed))
            retained_feedback[i]=candidate;
        frames[i]=retained_feedback[i];
    }
    return 1;
}

static void control_mode2_update(void)
{
    if (g_uart2_debug_mode!=RESERVED_UART2_CONTROL_MODE2_VALUE) {
        if (active) control_mode2_reset();
        return;
    }
    uint32_t now=imu_time_us();
    MotorFrame frames[2];
    if (!read_feedback(now,frames)) {
        control_mode2_reset();
        g_control_mode2_monitor.status=CONTROL2_BAD_FEEDBACK;
        return;
    }
    /* Raw joint-end rad/rad/s; left unchanged, right reversed exactly once. */
    float p[2]={frames[0].ang,-frames[1].ang};
    float v[2]={frames[0].speed,-frames[1].speed};
    if (!active) {
        if (g1_assist_v10_reset(&controller,p,v)!=0) {
            g_control_mode2_monitor.status=CONTROL2_NUMERIC_ERROR; return;
        }
        active=1U; previous_tick=now;
        g_control_mode2_monitor.status=CONTROL2_STARTING;
        return;
    }
    uint32_t elapsed=now-previous_tick;
    if (elapsed<PERIOD_US) return;
    g_control_mode2_monitor.last_period_us=elapsed;
    if (elapsed>g_control_mode2_monitor.max_period_us) g_control_mode2_monitor.max_period_us=elapsed;
    if (elapsed>MAX_PERIOD_US) {
        ++g_control_mode2_monitor.deadline_misses;
        control_mode2_reset();
        g_control_mode2_monitor.status=CONTROL2_BAD_PERIOD; return;
    }
    previous_tick=now;
    period_sum_us+=elapsed;
    ++period_samples;
    g_control_mode2_monitor.average_period_ms=(float)period_sum_us/(float)period_samples*0.001f;
    float torque[2]={0,0};
    int result=0;
    if (oscillation.latched) {
        g1_assist_v10_stop(&controller,0);
        (void)AssistOscillationMonitor_Update(&oscillation,p,v,0,&torque[0],&torque[1]);
        g_control_mode2_monitor.status=CONTROL2_SUPPRESSED;
    } else {
        if (!controller.ready) result=g1_assist_v10_reset(&controller,p,v);
        if (!result) result=g1_assist_v10_step(&controller,p,v,1,torque);
        if (result || !isfinite(torque[0]) || !isfinite(torque[1])) {
            control_mode2_reset();
            g_control_mode2_monitor.status=CONTROL2_NUMERIC_ERROR; return;
        }
        g_control_mode2_monitor.model_torque_nm[0]=torque[0];
        g_control_mode2_monitor.model_torque_nm[1]=torque[1];
        float scale=g_control_mode2_output_scale;
        if (!isfinite(scale) || scale<0.0f) {
            control_mode2_reset();
            g_control_mode2_monitor.status=CONTROL2_NUMERIC_ERROR; return;
        }
        for (unsigned i=0;i<2;++i)
            torque[i]=fmaxf(-TORQUE_LIMIT_NM,fminf(TORQUE_LIMIT_NM,torque[i]*scale));
        (void)AssistOscillationMonitor_Update(&oscillation,p,v,torque[0],&torque[0],&torque[1]);
        g_control_mode2_monitor.status=oscillation.latched ? CONTROL2_SUPPRESSED : CONTROL2_OK;
        if (oscillation.latched) g1_assist_v10_stop(&controller,0);
    }
    command[0]=torque[0]; command[1]=-torque[1]; /* model -> driver */
    calculated_torque[0]=torque[0]; calculated_torque[1]=torque[1];
    g_control_mode2_monitor.guard_latched=controller.guard.latched;
    g_control_mode2_monitor.oscillation_latched=oscillation.latched;
    ++g_control_mode2_monitor.ticks;
    float actual[2];
    (void)control_mode2_get_command(actual);
    g_control_mode2_monitor.driver_torque_nm[0]=actual[0];
    g_control_mode2_monitor.driver_torque_nm[1]=actual[1];
    /* Wall time of this control computation, including feedback and command
     * preparation; excludes idle polls, UART wire time and the period wait. */
    uint32_t duration=imu_time_us()-now;
    g_control_mode2_monitor.last_compute_us=duration;
    if (duration>g_control_mode2_monitor.max_compute_us) g_control_mode2_monitor.max_compute_us=duration;
    if (duration>MAX_PERIOD_US) {
        ++g_control_mode2_monitor.deadline_misses;
        control_mode2_reset(); g_control_mode2_monitor.status=CONTROL2_BAD_PERIOD;
        return;
    }
    compute_sum_us+=duration;
    ++compute_samples;
    g_control_mode2_monitor.average_compute_ms=(float)compute_sum_us/(float)compute_samples*0.001f;
}

/* Main-loop snapshot, 100Hz including fault/permission-off reports. Upload is
 * outside the timed computation. Busy UART drops a frame without blocking. */
void control_mode2_poll(void)
{
    static uint32_t publishing, last_publish, sequence;
    uint32_t started=imu_time_us();
    control_mode2_update();
    if (g_uart2_debug_mode!=RESERVED_UART2_CONTROL_MODE2_VALUE) {
        publishing=0U; return;
    }
    if (publishing && (uint32_t)(started-last_publish)<PERIOD_US) return;
    publishing=1U; last_publish=started;
    ControlMode2Telemetry t={0};
    t.start=CONTROL2_TELEMETRY_START; t.end=CONTROL2_TELEMETRY_END;
    t.version=1U; t.size=sizeof(t); t.sequence=sequence++;
    t.timestamp_us=started; t.status=g_control_mode2_monitor.status;
    MotorFrame frames[2];
    if (read_feedback(imu_time_us(),frames)) {
        t.flags|=1U;
        t.angle_rad[0]=frames[0].ang; t.angle_rad[1]=-frames[1].ang;
        t.speed_rad_s[0]=frames[0].speed; t.speed_rad_s[1]=-frames[1].speed;
    }
    if ((t.flags&1U) && active && (t.status==CONTROL2_OK || t.status==CONTROL2_SUPPRESSED)) {
        t.flags|=2U;
        t.calculated_torque_nm[0]=calculated_torque[0];
        t.calculated_torque_nm[1]=calculated_torque[1];
    }
    if (g_control_mode2_torque_enabled) t.flags|=4U;
    if (control_mode2_get_command(t.driver_torque_nm)) t.flags|=8U;
    if (g_control_mode2_monitor.guard_latched) t.flags|=16U;
    if (g_control_mode2_monitor.oscillation_latched) t.flags|=32U;
    t.compute_us=g_control_mode2_monitor.last_compute_us;
    t.period_us=g_control_mode2_monitor.last_period_us;
    t.output_scale=g_control_mode2_output_scale;
    t.average_compute_ms=g_control_mode2_monitor.average_compute_ms;
    (void)reserved_uart1_send_control2(&t);
}

int control_mode2_get_command(float driver_nm[2])
{
    MotorFrame frames[2];
    driver_nm[0]=driver_nm[1]=0;
    if (!g_control_mode2_torque_enabled || g_uart2_debug_mode!=RESERVED_UART2_CONTROL_MODE2_VALUE ||
        !active || g_control_mode2_monitor.status!=CONTROL2_OK ||
        (uint32_t)(imu_time_us()-previous_tick)>MAX_PERIOD_US ||
        !read_feedback(imu_time_us(),frames)) return 0;
    driver_nm[0]=command[0]; driver_nm[1]=command[1];
    return 1;
}

void control_mode2_fill_feedback(volatile td_mcu_frame_t *frame)
{
    normal_mode_fill_feedback(frame);
    frame->data4=frame->data5=frame->data6=frame->data7=0;
    frame->data6=g_control_mode2_monitor.average_compute_ms;
    MotorFrame feedback[2];
    if (read_feedback(imu_time_us(),feedback)) {
        frame->data4=feedback[0].ang*57.29577951308232f;
        frame->data5=feedback[1].ang*57.29577951308232f;
    }
    float actual[2]; (void)control_mode2_get_command(actual);
    frame->data8=actual[0]; frame->data9=actual[1];
}
