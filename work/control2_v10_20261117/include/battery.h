#ifndef BATTERY_H
#define BATTERY_H
#include <stdint.h>

/* Voltage bands for a conventional 4S Li-ion/LiPo pack, not a SOC estimate. */
#define BATTERY_REFERENCE_V 3.3f
#define BATTERY_DIVIDER_GAIN 11.0f
#define BATTERY_CALIBRATION_GAIN 1.0f

typedef struct {
    float voltage_v;
    float filtered_voltage_v;
    uint32_t adc_raw;
    uint32_t level;             /* 1=lowest, 4=highest; 0=invalid */
    uint32_t led_half_period_ms;
    uint32_t sample_count;
    uint32_t valid;
    int32_t last_error;         /* -1=calibration, -2=ready, -3=conversion timeout */
} BatteryData;
extern volatile BatteryData g_battery_data;
/* Call after imu_init(): shares its free-running 1MHz TIM2 timebase. */
void battery_init(void);
void battery_poll(void);
#endif
