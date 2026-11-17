#include "imu_debug_mode.h"
#include "imu.h"
void imu_debug_mode_fill_feedback(volatile td_mcu_frame_t *frame)
{

    /* 2026-11-13机身安装校正：XYZ=(-原Z,-原X,原Y)，X前/Y左/Z上。
     * data3=绕机身X轴的Roll[度]，data4=绕机身Y轴的Pitch[度]；
     * 均为右手正方向，由校正后的加速度/角速度重新融合计算。
     * data5..10的角速度/加速度也使用本映射，取一致快照上传。
     */
    ImuData imu;
    imu_get_body_data(&imu);
    /* IMU调试data2=2：先角度，再角速度，再加速度。不发送yaw。
     * data3=Roll[度]、data4=Pitch[度]；data5/6/7=角速度XYZ[rad/s]；
     * data8/9/10=加速度XYZ[m/s^2]，包含重力。Roll/Pitch由rad转度。
     * 8个float均用于IMU，本模式不发送电压，不沿用其他模式残留字段。
     */
    const float rad_to_deg = 57.29577951308232f;
    frame->data3 = imu.angle_rad.roll * rad_to_deg;
    frame->data4 = imu.angle_rad.pitch * rad_to_deg;
    frame->data5 = imu.gyro_rad_s.x;
    frame->data6 = imu.gyro_rad_s.y;
    frame->data7 = imu.gyro_rad_s.z;
    frame->data8 = imu.accel_m_s2.x;
    frame->data9 = imu.accel_m_s2.y;
    frame->data10 = imu.accel_m_s2.z;
}
