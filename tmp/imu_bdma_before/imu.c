#include "stm32h7xx.h"
#include "imu.h"
#include "imu_bus.h"
#include "icm_imu_i2c.h"
#include "mahony.h"

volatile ImuData g_imu_data;
/* 竖装坐标按实测修正，见下方映射；保留g_imu_data的原传感器坐标。 */
volatile ImuData g_imu_body_data;
static MahonyState body_attitude;
static icm_imu_t sensor;
static uint32_t previous_read, retry_at;
static volatile uint32_t stream_enabled;
static unsigned consecutive_errors;
static int ready, have_previous;

void TIM7_IRQHandler(void)
{
    if (TIM7->SR & TIM_SR_UIF) {
        TIM7->SR = 0U;
        if (stream_enabled) imu_bus_schedule_sample(sensor.addr7);
    }
    __DSB();
}

static void sample_timer_init(void)
{
    /* APB1 timer kernel=275MHz; /275 then /1000 gives 1ms updates. */
    RCC->APB1LENR |= RCC_APB1LENR_TIM7EN;
    (void)RCC->APB1LENR;
    TIM7->CR1 = TIM_CR1_ARPE;
    TIM7->CR2 = 0U;
    TIM7->DIER = 0U;
    TIM7->PSC = 274U;
    TIM7->ARR = 999U;
    TIM7->CNT = 0U;
    TIM7->EGR = TIM_EGR_UG;
    TIM7->SR = 0U;
    NVIC_SetPriority(TIM7_IRQn, 5U);
    NVIC_ClearPendingIRQ(TIM7_IRQn);
    NVIC_EnableIRQ(TIM7_IRQn);
    TIM7->DIER = TIM_DIER_UIE;
    TIM7->CR1 |= TIM_CR1_CEN;
}

static int start_sensor(void)
{
    const icm_imu_bus_t bus = {0, imu_bus_read, imu_bus_write, imu_delay_us};
    int result = imu_bus_recover();
    if (result != 0) return result;
    const uint8_t addresses[] = {0x69U, 0x68U};
    for (unsigned i = 0; i < 2U; ++i) {
        /* Restore bank 0 if an earlier initialization was interrupted. */
        uint8_t id;
        result = imu_bus_write(0, addresses[i], 0x76U, 0U);
        if (result != 0) continue;
        result = imu_bus_read(0, addresses[i], 0x75U, &id, 1U);
        if (result != 0) continue;
        if (id != 0x47U) { result = ICM_IMU_ERR_ID; continue; }
        result = icm_imu_init(&sensor, &bus, addresses[i]);
        if (result == 0) return 0;
    }
    return result;
}

void imu_init(void)
{
    imu_bus_init();
    imu_delay_us(0, 100000U);
    sample_timer_init();
    /* Initialization/retries are handled by the main-loop service. */
    retry_at = imu_time_us();
}

void imu_get_data(ImuData *out)
{
    if (out == 0) return;
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    *out = g_imu_data;
    __set_PRIMASK(saved);
}

void imu_get_body_data(ImuData *out)
{
    if (out == 0) return;
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    *out = g_imu_body_data;
    __set_PRIMASK(saved);
}

void imu_poll(void)
{
    uint32_t now = imu_time_us();
    if (!ready) {
        if ((int32_t)(now - retry_at) < 0) return;
        int result = start_sensor();
        g_imu_data.last_error = result;
        g_imu_data.valid = 0U;
        g_imu_data.angle_valid = 0U;
        g_imu_body_data.valid = 0U;
        g_imu_body_data.angle_valid = 0U;
        g_imu_body_data.last_error = result;
        if (result != 0) { retry_at = imu_time_us() + 500000U; return; }
        ready = 1;
        have_previous = 0;
        consecutive_errors = 0;
        mahony_reset();
        mahony_state_reset(&body_attitude);
        __DMB();
        stream_enabled = 1U;
        return; /* First transfer starts at the next TIM7 interrupt. */
    }
    uint8_t bytes[12];
    int result;
    if (!imu_bus_take_sample(bytes, &now, &result)) return;
    icm_imu_sample_t sample;
    if (result == 0) result = icm_imu_decode_motion(bytes, &sample);
    if (result != 0) {
        g_imu_data.valid = 0U;
        g_imu_data.angle_valid = 0U;
        g_imu_body_data.valid = 0U;
        g_imu_body_data.angle_valid = 0U;
        g_imu_body_data.last_error = result;
        g_imu_data.last_error = result;
        if (++consecutive_errors >= 5U) {
            uint32_t saved = __get_PRIMASK();
            __disable_irq();
            stream_enabled = 0U;
            __DMB();
            imu_bus_cancel_sample();
            __set_PRIMASK(saved);
            ready = 0;
            retry_at = imu_time_us() + 500000U;
        }
        return;
    }
    consecutive_errors = 0;
    uint32_t elapsed = have_previous ? (uint32_t)(now - previous_read) : 1000U;
    if (elapsed == 0U || elapsed > 10000U) {
        mahony_reset();
        mahony_state_reset(&body_attitude);
        elapsed = 1000U;
    }
    previous_read = now;
    have_previous = 1;
    float angles[3];
    int angle_valid = mahony_update(sample.accel_g, sample.gyro_dps, elapsed * 1.0e-6f, angles);
    ImuData data = {0};
    data.accel_m_s2.x = sample.accel_g[0] * 9.80665f;
    data.accel_m_s2.y = sample.accel_g[1] * 9.80665f;
    data.accel_m_s2.z = sample.accel_g[2] * 9.80665f;
    data.gyro_rad_s.x = sample.gyro_dps[0] * 0.01745329252f;
    data.gyro_rad_s.y = sample.gyro_dps[1] * 0.01745329252f;
    data.gyro_rad_s.z = sample.gyro_dps[2] * 0.01745329252f;
    if (angle_valid) {
        data.angle_rad.roll = angles[0] * 0.01745329252f;
        data.angle_rad.pitch = angles[1] * 0.01745329252f;
        data.angle_rad.yaw = angles[2] * 0.01745329252f;
    }
    data.valid = 1U;
    data.angle_valid = (uint32_t)angle_valid;
    data.timestamp_us = now;
    data.sample_count = g_imu_data.sample_count + 1U;
    /* 最终安装映射（用户实测确认可用）：
     * 机身(X,Y,Z)=(传感器Z,传感器X,传感器Y)。
     * 相对上一版(-传感器Z,传感器X,-传感器Y)，X/Z同时反向、Y不变，
     * 相当于坐标系绕Y轴旋转180度，保持右手坐标系。
     * 用户测试的直立位置：Roll由约180度变为约0度；Pitch正负反转。
     * 按用户测试时的同一观察方向：Pitch顺时针为负、逆时针为正；
     * “顺/逆时针”依赖观察方向，不作为任意视角下的通用定义。
     * 加速度和角速度使用相同旋转，并用独立滤波状态解算机身姿态；
     * 欧拉角不能直接按向量交换/取反。原始结构体保留原坐标，IMU调试串口上传本映射结构体。
     */
    float body_accel[3] = {sample.accel_g[2], sample.accel_g[0], sample.accel_g[1]};
    float body_gyro[3] = {sample.gyro_dps[2], sample.gyro_dps[0], sample.gyro_dps[1]};
    float body_angles[3];
    int body_valid = mahony_state_update(&body_attitude, body_accel, body_gyro,
                                         elapsed * 1.0e-6f, body_angles);
    ImuData body = data;
    body.accel_m_s2.x = data.accel_m_s2.z;
    body.accel_m_s2.y = data.accel_m_s2.x;
    body.accel_m_s2.z = data.accel_m_s2.y;
    body.gyro_rad_s.x = data.gyro_rad_s.z;
    body.gyro_rad_s.y = data.gyro_rad_s.x;
    body.gyro_rad_s.z = data.gyro_rad_s.y;
    body.angle_valid = (uint32_t)body_valid;
    body.angle_rad.roll = body_valid ? body_angles[0] * 0.01745329252f : 0.0f;
    body.angle_rad.pitch = body_valid ? body_angles[1] * 0.01745329252f : 0.0f;
    body.angle_rad.yaw = body_valid ? body_angles[2] * 0.01745329252f : 0.0f;
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    g_imu_body_data = body;
    g_imu_data = data;
    __set_PRIMASK(saved);
}
