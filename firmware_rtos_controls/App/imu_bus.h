#ifndef IMU_BUS_H
#define IMU_BUS_H
#include <stdint.h>
void imu_bus_init(void);
uint32_t imu_time_us(void);
int imu_bus_recover(void);
int imu_bus_read(void *ctx, uint8_t address, uint8_t reg, uint8_t *data, uint16_t size);
int imu_bus_write(void *ctx, uint8_t address, uint8_t reg, uint8_t value);
void imu_delay_us(void *ctx, uint32_t us);
/* TIM7 checks for a stalled transfer each tick and starts a sample only when due. */
void imu_bus_service_sample(uint8_t address, int sample_due);
/* Main-loop consumer: 0=no completion, 1=completion (check result). */
int imu_bus_take_sample(uint8_t data[12], uint32_t *timestamp_us, int *result);
void imu_bus_cancel_sample(void);
#endif
