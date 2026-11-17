#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "control_mode1.h"
#include "reserved_uart1.h"
#include "reserved_uart2.h"
#include "motor_bus.h"
#include "imu.h"
/* Windows TCC CRT exposes the double-precision math entry points. */
float fabsf(float x){return (float)fabs(x);}
float cosf(float x){return (float)cos(x);}
float sinf(float x){return (float)sin(x);}
float sqrtf(float x){return (float)sqrt(x);}
float atan2f(float y,float x){return (float)atan2(y,x);}
volatile uint32_t g_uart2_debug_mode;
volatile MotorData g_motor_left,g_motor_right;
volatile ImuData g_imu_body_data;
volatile uint32_t g_motor_zero_switch,g_motor_zero_state;
static uint32_t now=100000,sends;
static MotorFrame motors[2];
static int have_feedback[2]={1,1};
static ControlMode1Observation sent;
uint32_t imu_time_us(void){return now;}
void imu_get_body_data(ImuData *out){*out=g_imu_body_data;}
int motor_get_feedback(uint32_t side,MotorFrame *out){*out=motors[side];return have_feedback[side];}
void normal_mode_fill_feedback(volatile td_mcu_frame_t *f){(void)f;}
int reserved_uart1_send_observation(const ControlMode1Observation *f){sent=*f;++sends;return 1;}
static void fresh(void){
    g_imu_body_data.timestamp_us=now;
    g_motor_left.rx_timestamp_us=g_motor_right.rx_timestamp_us=now;
}
static void tick(void){now+=20000;fresh();control_mode1_poll();}
static void near(float a,float b){assert(fabsf(a-b)<0.0001f);}
int main(void){
    float obs[65]={0}; ControlMode1Observation f;
    assert(sizeof(f)==52 && offsetof(ControlMode1Observation,end_flag)==48);
    for(unsigned i=0;i<11;++i)obs[54+i]=(float)(i+1)/10;
    control_mode1_observation_from_obs(&f,obs);
    assert(f.start_flag==0xAAAAAAAAU && f.end_flag==0xEEEEEEEEU);
    near(f.joint_angle_rad[0],0.1f*S2_PI);near(f.joint_angle_rad[1],0.2f*S2_PI);
    near(f.joint_speed_rad_s[0],0.3f*4*S2_PI);near(f.joint_speed_rad_s[1],0.4f*4*S2_PI);
    near(f.previous_torque_nm[0],20);near(f.previous_torque_nm[1],24);
    near(f.body_tilt_rad[0],0.7f*S2_PI);near(f.body_tilt_rad[1],0.8f*S2_PI);
    for(unsigned i=0;i<3;++i)near(f.body_gyro_rad_s[i],obs[62+i]*4*S2_PI);
    assert(!g_control_mode1_observation_enabled);
    g_uart2_debug_mode=1;g_imu_body_data.valid=g_imu_body_data.angle_valid=1;
    g_imu_body_data.gyro_rad_s.x=2;g_imu_body_data.gyro_rad_s.y=3;g_imu_body_data.gyro_rad_s.z=4;
    motors[0].mode=motors[1].mode=MOTOR_TORQUE;
    motors[0].ang=0.2f;motors[1].ang=0.3f;motors[0].speed=0.4f;motors[1].speed=0.5f;
    fresh();control_mode1_poll();tick();assert(sends==0);
    control_mode1_set_observation_enabled(1);tick();assert(sends==1);
    assert(g_control_mode1_status==1);
    near(sent.joint_angle_rad[0],0.2f);near(sent.joint_angle_rad[1],-0.3f);
    near(sent.joint_speed_rad_s[0],0.4f);near(sent.joint_speed_rad_s[1],-0.5f);
    near(sent.body_gyro_rad_s[0],-3);near(sent.body_gyro_rad_s[1],2);near(sent.body_gyro_rad_s[2],4);
    near(sent.previous_torque_nm[0],0);near(sent.previous_torque_nm[1],0);
    for(unsigned i=0;i<10;++i)tick();assert(g_control_mode1_status==0);
    float last[2]={g_control_mode1_monitor.model_torque_nm[0],g_control_mode1_monitor.model_torque_nm[1]};
    tick();near(sent.previous_torque_nm[0],last[0]);near(sent.previous_torque_nm[1],last[1]);
    unsigned count=sends;control_mode1_set_observation_enabled(0);tick();assert(sends==count);
    control_mode1_set_observation_enabled(1);now+=40000;control_mode1_poll();assert(sends==count);
    tick();tick();assert(sends==count+1 && g_control_mode1_status==1);
    for(unsigned i=0;i<10;++i)tick();assert(g_control_mode1_status==0);
    /* Age, skew, IMU flags/errors and motor mode/error do not reset policy. */
    g_imu_body_data.valid=g_imu_body_data.angle_valid=0;g_imu_body_data.last_error=-12;
    g_imu_body_data.timestamp_us=0;g_motor_left.rx_timestamp_us=0;g_motor_right.rx_timestamp_us=now;
    motors[0].mode=MOTOR_ZERO;motors[1].error_flag=123;
    now+=20000;control_mode1_poll();assert(g_control_mode1_status==0);
    near(sent.joint_angle_rad[0],.2f);near(sent.joint_angle_rad[1],-.3f);
    have_feedback[0]=have_feedback[1]=0;motors[0].ang=NAN;g_imu_body_data.gyro_rad_s.x=NAN;
    now+=20000;control_mode1_poll();assert(g_control_mode1_status==0);
    near(sent.joint_angle_rad[0],.2f);near(sent.body_gyro_rad_s[1],2);
    have_feedback[0]=have_feedback[1]=1;motors[0].ang=.2f;
    now+=20000;control_mode1_poll();assert(g_control_mode1_status==0);
    g_motor_zero_switch=1;control_mode1_poll();assert(g_control_mode1_status<0);
    float command[2];assert(!control_mode1_get_command(command));g_motor_zero_switch=0;
    g_motor_zero_state=MOTOR_ZERO_WAIT_FEEDBACK;control_mode1_poll();assert(g_control_mode1_status<0);
    g_motor_zero_state=MOTOR_ZERO_DONE;control_mode1_poll();tick();assert(g_control_mode1_status==1);
    g_uart2_debug_mode=0;control_mode1_poll();assert(!g_control_mode1_observation_enabled);
    puts("PASS: 52-byte layout, 11 SI fields, model signs, warmup, previous torque, upload gating, feedback-error bypass, retained finite inputs, zeroing, timing check, exit");
    return 0;
}
