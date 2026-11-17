#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "control_mode3.h"
#include "reserved_uart2.h"
#include "motor_bus.h"
#include "control_mode2_telemetry.h"
static ControlMode2Telemetry telemetry;
int reserved_uart1_send_control2(const ControlMode2Telemetry *t){telemetry=*t;return 1;}
volatile uint32_t g_uart2_debug_mode,g_motor_zero_switch,g_motor_zero_state;
volatile MotorData g_motor_left,g_motor_right;
static uint32_t now=100000;
static MotorFrame motor[2];
static int valid[2]={1,1};
uint32_t imu_time_us(void){return now;}
int motor_get_feedback(unsigned s,MotorFrame *f){*f=motor[s];return valid[s];}
void normal_mode_fill_feedback(volatile td_mcu_frame_t *f){*f=(td_mcu_frame_t){0};}
static void fresh(void){g_motor_left.rx_timestamp_us=g_motor_right.rx_timestamp_us=now;}
int main(void){
 float a[2]; td_mcu_frame_t f;
 assert(reserved_uart2_accept_mode(3)==3);
 motor[0].mode=motor[1].mode=MOTOR_TORQUE;fresh();g_uart2_debug_mode=3;
 assert(!control_mode3_get_command(a));control_mode3_set_torque_enabled(1);
 assert(control_mode3_get_command(a)&&a[0]==0&&a[1]==0);
 for(unsigned n=1;n<=400;n++){now+=10000;fresh();assert(control_mode3_get_command(a));assert(fabsf(a[0]-3*sinf(3.14159265359f*n*.01f))<.0001f);assert(a[1]==0);}
 control_mode3_set_torque_enabled(0);assert(!control_mode3_get_command(a));control_mode3_set_torque_enabled(1);assert(control_mode3_get_command(a)&&a[0]==0);
 now+=40000;assert(control_mode3_get_command(a));assert(g_control_mode3_monitor.status&CONTROL3_LEFT_OLD);
 assert(fabsf(a[0]-3*sinf(.04f*3.14159265359f))<.0001f);fresh();
 uint32_t resets=g_control_mode3_monitor.resets;
 g_motor_right.rx_timestamp_us=now-16000;assert(control_mode3_get_command(a));assert(g_control_mode3_monitor.status&CONTROL3_SKEW);fresh();
 motor[1].error_flag=1;assert(control_mode3_get_command(a));assert(g_control_mode3_monitor.status&CONTROL3_RIGHT_ERROR);motor[1].error_flag=0;
 motor[0].mode=0;assert(control_mode3_get_command(a));assert(g_control_mode3_monitor.status&CONTROL3_LEFT_MODE);motor[0].mode=MOTOR_TORQUE;
 valid[1]=0;assert(control_mode3_get_command(a));valid[1]=1;
 motor[0].ang=NAN;assert(control_mode3_get_command(a));assert(g_control_mode3_monitor.status&CONTROL3_LEFT_NUMERIC);
 control_mode3_poll();assert(!(telemetry.flags&1));assert((telemetry.flags&14)==14);assert(isfinite(telemetry.driver_torque_nm[0]));
 motor[0].ang=0;assert(control_mode3_get_command(a));assert(g_control_mode3_monitor.status==0);assert(g_control_mode3_monitor.resets==resets);
 for(unsigned z=1;z<6;z++){g_motor_zero_state=z;assert(!control_mode3_get_command(a));}g_motor_zero_state=6;g_motor_zero_switch=1;assert(!control_mode3_get_command(a));g_motor_zero_switch=0;assert(control_mode3_get_command(a)&&a[0]==0);
 g_uart2_debug_mode=0;assert(!control_mode3_get_command(a));g_uart2_debug_mode=3;assert(control_mode3_get_command(a)&&a[0]==0);
 now=0xfffffff0U;fresh();control_mode3_reset();assert(control_mode3_get_command(a));now+=10000;fresh();assert(control_mode3_get_command(a));assert(fabsf(a[0]-3*sinf(.01f*3.14159265359f))<.0001f);
 control_mode3_fill_feedback(&f);assert(f.data8==a[0]&&f.data9==a[1]);assert(!control_mode3_set_parameters(NAN,2));assert(!control_mode3_set_parameters(16,2));
 assert(!control_mode3_set_parameters(4,0));assert(!control_mode3_set_parameters(4,61));
 assert(control_mode3_set_parameters(4,1));assert(control_mode3_get_command(a)&&a[0]==0);
 now+=250000;fresh();assert(control_mode3_get_command(a));assert(fabsf(a[0]-4)<.0001f&&a[1]==0);
 assert(control_mode3_set_parameters(4,1));assert(control_mode3_get_command(a)&&fabsf(a[0]-4)<.0001f);
 control_mode3_poll();assert(telemetry.version==3&&telemetry.size==80&&(telemetry.flags&15)==15);
 assert(telemetry.output_scale==4&&telemetry.average_compute_ms==1);
 assert(telemetry.calculated_torque_nm[0]==4&&telemetry.calculated_torque_nm[1]==0);
 assert(telemetry.driver_torque_nm[0]==4&&telemetry.driver_torque_nm[1]==0);
 FILE *fp=fopen("work/control3_20261117/telemetry_fixture.bin","wb");assert(fp);fwrite(&telemetry,1,sizeof(telemetry),fp);fclose(fp);
 control_mode3_set_torque_enabled(0);now+=10000;fresh();control_mode3_poll();assert(!(telemetry.flags&14));
 assert(control_mode3_set_parameters(0,.1f));control_mode3_set_torque_enabled(1);assert(control_mode3_get_command(a)&&a[0]==0);
 puts("Mode3 tests PASS: waveform, sign, permission, feedback diagnostics without suppression, zeroing, restart, timer wrap, telemetry");return 0;
}
