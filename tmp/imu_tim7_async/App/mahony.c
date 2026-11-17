#include "mahony.h"
#include <math.h>

#define RAD_PER_DEG 0.01745329252f
static float q[4], integral[3];
static int initialized;

static float limit(float value, float bound)
{
    return value < -bound ? -bound : (value > bound ? bound : value);
}

void mahony_reset(void)
{
    initialized = 0;
    for (unsigned i = 0; i < 3; ++i) integral[i] = 0.0f;
}

int mahony_update(const float a[3], const float gyro[3], float dt, float angles[3])
{
    float norm2 = a[0]*a[0] + a[1]*a[1] + a[2]*a[2];
    int gravity_ok = norm2 >= 0.85f*0.85f && norm2 <= 1.15f*1.15f;
    if (!initialized) {
        if (!gravity_ok) return 0;
        float roll = 0.5f*atan2f(a[1], a[2]);
        float pitch = 0.5f*atan2f(-a[0], sqrtf(a[1]*a[1] + a[2]*a[2]));
        float cr = cosf(roll), sr = sinf(roll), cp = cosf(pitch), sp = sinf(pitch);
        q[0]=cr*cp; q[1]=sr*cp; q[2]=cr*sp; q[3]=-sr*sp;
        initialized = 1;
    } else {
        float w=q[0], x=q[1], y=q[2], z=q[3];
        float error[3] = {0};
        if (gravity_ok) {
            float inv = 1.0f/sqrtf(norm2);
            float vx=2*(x*z-w*y), vy=2*(w*x+y*z), vz=1-2*(x*x+y*y);
            error[0]=(a[1]*vz-a[2]*vy)*inv;
            error[1]=(a[2]*vx-a[0]*vz)*inv;
            error[2]=(a[0]*vy-a[1]*vx)*inv;
        }
        float half_step[3];
        for (unsigned i=0; i<3; ++i) {
            integral[i]=limit(integral[i]+0.05f*error[i]*dt, 0.2f);
            half_step[i]=(gyro[i]*RAD_PER_DEG+2.0f*error[i]+integral[i])*dt*0.5f;
        }
        float u=half_step[0], v=half_step[1], r=half_step[2];
        q[0]=w-x*u-y*v-z*r; q[1]=x+w*u+y*r-z*v;
        q[2]=y+w*v+z*u-x*r; q[3]=z+w*r+x*v-y*u;
        float inv=1.0f/sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
        for (unsigned i=0; i<4; ++i) q[i]*=inv;
    }
    float w=q[0], x=q[1], y=q[2], z=q[3];
    angles[0]=atan2f(2*(w*x+y*z), 1-2*(x*x+y*y))/RAD_PER_DEG;
    angles[1]=asinf(limit(2*(w*y-z*x),1.0f))/RAD_PER_DEG;
    angles[2]=atan2f(2*(w*z+x*y), 1-2*(y*y+z*z))/RAD_PER_DEG;
    return 1;
}
