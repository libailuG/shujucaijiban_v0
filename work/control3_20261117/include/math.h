#include "C:/Users/libai/Desktop/X1/imu_caiji/zhuban_caiji/work/cascade_test/tcc/include/math.h"
/* Host-only wrappers avoid legacy TCC float signbit/min/max implementation. */
static float host_min(float a,float b){return a<b?a:b;}
static float host_max(float a,float b){return a>b?a:b;}
#define fminf host_min
#define fmaxf host_max
static float host_sin(float x){return (float)sin((double)x);}
static float host_cos(float x){return (float)cos((double)x);}
static float host_exp(float x){return (float)exp((double)x);}
static float host_abs(float x){return x<0?-x:x;}
#define sinf host_sin
#define cosf host_cos
#define expf host_exp
#define fabsf host_abs
