#ifndef CONTROL_MODE1_OBSERVATION_H
#define CONTROL_MODE1_OBSERVATION_H
#include <stdint.h>
#include "control1_policy/s2_deploy.h"

/* UART4: little-endian uint32 + 11 FP32 + uint32, model coordinates, SI units.
 * Only the latest observation slice is exported, before mean/std normalization.
 * Previous torque is the policy's limited active command, not measured torque. */
typedef struct {
    uint32_t start_flag;
    float joint_angle_rad[2];
    float joint_speed_rad_s[2];
    float previous_torque_nm[2];
    float body_tilt_rad[2]; /* forward/backward, left/right */
    float body_gyro_rad_s[3]; /* model waist XYZ */
    uint32_t end_flag;
} ControlMode1Observation;
typedef char ControlMode1ObservationSizeCheck[(sizeof(ControlMode1Observation)==52U)?1:-1];

static inline void control_mode1_observation_from_obs(ControlMode1Observation *f,
                                                      const float obs[S2_OBS_DIM])
{
    f->start_flag=0xAAAAAAAAU;
    for (unsigned i=0;i<2;++i) {
        f->joint_angle_rad[i]=obs[54+i]*S2_PI;
        f->joint_speed_rad_s[i]=obs[56+i]*(4.0f*S2_PI);
        f->previous_torque_nm[i]=obs[58+i]*40.0f;
        f->body_tilt_rad[i]=obs[60+i]*S2_PI;
    }
    for (unsigned i=0;i<3;++i) f->body_gyro_rad_s[i]=obs[62+i]*(4.0f*S2_PI);
    f->end_flag=0xEEEEEEEEU;
}
#endif
