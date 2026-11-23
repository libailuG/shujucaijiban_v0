/* Offline synthetic encoder example. Does not connect to motors. */
#include "hip_assist_caiji_v0_policy.h"
#include <math.h>
#include <stdio.h>

int main(void)
{
    /* Static state avoids putting the history/guard workspace on a small MCU stack. */
    static HipAssistCaijiV0Controller controller;
    float q[2]={0,0}, dq[2]={0,0}, torque[2]={0,0};
    if(hip_assist_caiji_v0_reset(&controller,q,dq,5,2)!=0) return 1;
    puts("time_s,left_nm,right_nm,grade,desired_profile,stage,reward_profile");
    for(int i=0;i<2000;++i) {
        double t=(double)i*.01;
        q[0]=.35f*sinf((float)(2.6*t));q[1]=-q[0];
        dq[0]=.91f*cosf((float)(2.6*t));dq[1]=-dq[0];
        if(i==400 && hip_assist_caiji_v0_set_precontact_profile(&controller,1)!=0) return 2;
        if(i==800 && hip_assist_caiji_v0_set_precontact_profile(&controller,3)!=0) return 2;
        if(i==1000 && hip_assist_caiji_v0_set_grade(&controller,7)!=0) return 2;
        if(i==1500 && hip_assist_caiji_v0_set_grade(&controller,0)!=0) return 2;
        int result=hip_assist_caiji_v0_step_timed(&controller,q,dq,1,t,torque);
        if(result!=0) {
            hip_assist_caiji_v0_stop(&controller,torque);
            /* Hardware integration must actually transmit zero/disable here. */
            return 3;
        }
        printf("%.2f,%.7f,%.7f,%u,%u,%u,%d\n",t,(double)torque[0],(double)torque[1],
            controller.current_grade,controller.precontact_profile,controller.walking_stage,
            hip_assist_caiji_v0_reward_precontact_profile(&controller));
    }
    hip_assist_caiji_v0_stop(&controller,torque);
    return 0;
}
