#ifndef MAHONY_H
#define MAHONY_H
void mahony_reset(void);
/* Sensor axes, acceleration in g, angular rate in deg/s, dt in seconds.
 * Returns 1 once gravity has supplied the initial tilt. ZYX Euler output
 * is roll, pitch, relative yaw in degrees, for a Z-up reference frame. */
int mahony_update(const float accel[3], const float gyro[3], float dt, float angles[3]);
#endif
