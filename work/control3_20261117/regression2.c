#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "control_mode2.h"
#include "control2_policy/g1_assist_v10_policy.h"
#include "control2_policy/assist_oscillation_monitor.h"
#include "reserved_uart2.h"
#include "motor_bus.h"
#include "control_mode2_telemetry.h"




volatile uint32_t g_uart2_debug_mode,g_motor_zero_switch,g_motor_zero_state;
volatile MotorData g_motor_left,g_motor_right;
static uint32_t now=100000, read_cost_us;
static MotorFrame motor[2];
static ControlMode2Telemetry telemetry;
int reserved_uart1_send_control2(const ControlMode2Telemetry *f){telemetry=*f;return 1;}
uint32_t imu_time_us(void){return now;}
int motor_get_feedback(unsigned side,MotorFrame *f){*f=motor[side];now+=read_cost_us;return 1;}
void normal_mode_fill_feedback(volatile td_mcu_frame_t *f){*f=(td_mcu_frame_t){0};}
static void fresh(void){g_motor_left.rx_timestamp_us=g_motor_right.rx_timestamp_us=now;}
int main(void){
    float actual[2],p[2],v[2],expected[2];
    static G1AssistV10Controller reference;
    int nonzero=0;
    assert(reserved_uart2_accept_mode(2)==2);
    assert(reserved_uart2_accept_mode(12)==12);
    assert(reserved_uart2_accept_mode(3)==3);
    for(unsigned level=0;level<8;++level){
        control_mode2_reset(); g_uart2_debug_mode=2;
        control_mode2_set_torque_enabled(1); assert(control_mode2_set_output_level(level));
        for(unsigned n=0;n<300;++n){
            now+=10000; fresh();
            for(unsigned s=0;s<2;++s){
                float phase=n*.08f+s*3.14159265f;
                p[s]=.4f*sinf(phase);v[s]=3.2f*cosf(phase);
                motor[s]=(MotorFrame){.mode=MOTOR_TORQUE,.ang=p[s]*(s?-1:1),.speed=v[s]*(s?-1:1)};
            }
            if(!n){control_mode2_poll();assert(g_control_mode2_monitor.status==CONTROL2_STARTING);continue;}
            if(n==1)assert(g1_assist_v10_reset(&reference,p,v)==0);
            assert(!g1_assist_v10_step(&reference,p,v,1,expected));
            control_mode2_poll();assert(control_mode2_get_command(actual));
            assert(fabsf(actual[0]-expected[0]*g_control_mode2_output_scale)<.0001f);
            assert(telemetry.size==80 && telemetry.version==1 && (telemetry.flags&15)==15);
            assert(fabsf(telemetry.angle_rad[1]-p[1])<.0001f);
            assert(fabsf(telemetry.speed_rad_s[0]-v[0])<.0001f);
            assert(fabsf(telemetry.calculated_torque_nm[0]-actual[0])<.0001f);
            assert(fabsf(telemetry.calculated_torque_nm[1]+actual[1])<.0001f);
            assert(fabsf(actual[1]+expected[1]*g_control_mode2_output_scale)<.0001f);
            assert(fabsf(actual[0])<=15 && fabsf(actual[1])<=15);
            assert(fabsf(actual[0]-actual[1])<.0001f); /* driver signs */
            nonzero|=fabsf(actual[0])>.01f;
            control_mode2_set_torque_enabled(0);assert(!control_mode2_get_command(actual));assert(actual[0]==0&&actual[1]==0);
            control_mode2_set_torque_enabled(1);
        }
    }
    assert(nonzero);assert(!control_mode2_set_output_level(8));
    control_mode2_set_torque_enabled(0);now+=10000;fresh();control_mode2_poll();
    assert((telemetry.flags&15)==3 && telemetry.driver_torque_nm[0]==0 && telemetry.driver_torque_nm[1]==0);
    FILE *wire=fopen("work/control2_v10_20261117/telemetry_fixture.bin","wb");assert(wire);assert(fwrite(&telemetry,1,80,wire)==80);fclose(wire);
    control_mode2_set_torque_enabled(1);
    td_mcu_frame_t f;control_mode2_fill_feedback(&f);assert(f.data2==0); assert(f.data6==0); assert(fabsf(g_control_mode2_monitor.average_period_ms-10.0f)<.001f); /* dispatcher sets mode */
    now+=31000;assert(!control_mode2_get_command(actual));control_mode2_poll();assert(g_control_mode2_monitor.status==CONTROL2_BAD_FEEDBACK);
    control_mode2_fill_feedback(&f);assert(f.data6==0);
    fresh();control_mode2_poll();assert(g_control_mode2_monitor.status==CONTROL2_STARTING);
    now+=10000;fresh();control_mode2_poll();now+=11000;fresh();control_mode2_poll();
    control_mode2_fill_feedback(&f);assert(f.data6==0); assert(fabsf(g_control_mode2_monitor.average_period_ms-10.5f)<.001f);
    now+=12000;fresh();control_mode2_poll();assert(g_control_mode2_monitor.status==CONTROL2_BAD_PERIOD);
    control_mode2_poll();now+=10000;fresh();g_motor_left.rx_timestamp_us-=16000;control_mode2_poll();assert(g_control_mode2_monitor.status==CONTROL2_BAD_FEEDBACK);
    fresh();motor[0].ang=NAN;control_mode2_poll();assert(g_control_mode2_monitor.status==CONTROL2_BAD_FEEDBACK);motor[0].ang=0;
    g_motor_zero_switch=1;control_mode2_poll();assert(!control_mode2_get_command(actual));g_motor_zero_switch=0;
    g_motor_zero_state=MOTOR_ZERO_WAIT_FEEDBACK;control_mode2_poll();assert(!control_mode2_get_command(actual));g_motor_zero_state=MOTOR_ZERO_DONE;
    fresh();control_mode2_poll();g_uart2_debug_mode=0;control_mode2_poll();assert(g_control_mode2_monitor.status==CONTROL2_DISABLED);
    /* Synthetic feedback cost proves data6 measures work, not the 10ms period. */
    g_uart2_debug_mode=2;control_mode2_set_torque_enabled(1);fresh();control_mode2_poll();
    read_cost_us=100;now+=10000;fresh();control_mode2_poll();
    assert(g_control_mode2_monitor.last_compute_us==400);
    assert(fabsf(g_control_mode2_monitor.average_compute_ms-.4f)<.001f);
    read_cost_us=200;now+=10000;fresh();control_mode2_poll();read_cost_us=0;
    assert(g_control_mode2_monitor.last_compute_us==800);
    control_mode2_fill_feedback(&f);assert(fabsf(f.data6-.6f)<.001f);
    control_mode2_reset();control_mode2_fill_feedback(&f);assert(f.data6==0);
    static AssistOscillationMonitor_t monitor;
    float a=3,b=-3;
    for(unsigned n=0;n<150;++n){
        float q[2]={.15f*sinf(n*.09424778f),-.15f*sinf(n*.09424778f)};
        float vel[2]={0,0}; a=3;b=-3;
        AssistOscillationMonitor_Update(&monitor,q,vel,3,&a,&b);
    }
    assert(monitor.latched && a==0 && b==0);
    for(unsigned n=0;n<20;++n){float q[2]={0,0};AssistOscillationMonitor_Update(&monitor,q,q,0,&a,&b);}
    assert(!monitor.latched);
    p[0]=p[1]=v[0]=v[1]=0;
    assert(!g1_assist_v10_reset(&reference,p,v));
    reference.guard.count=150;reference.guard.previous=3;
    for(unsigned n=0;n<150;++n){reference.guard.torques[n]=2;reference.guard.reversals[n][0]=n<8?1:0;}
    assert(!g1_assist_v10_step(&reference,p,v,1,expected));
    assert(reference.guard.latched && fabsf(expected[0]-2.2f)<.0001f);
    puts("PASS: eight reference levels, left/right mapping, torque gate, bounds, stale/skew/NaN, deadline, zeroing, exit, internal guard and external suppression/rearm");
    return 0;
}
