#ifndef ICM_IMU_I2C_H
#define ICM_IMU_I2C_H

/* Portable ICM-42670-P / ICM-42688-P I2C reference driver, v1.0.
 * Fixed profile: accel +/-16 g, gyro +/-2000 dps, low-noise mode.
 * 42670: 1600 Hz ODR. 42688: 32000 Hz ODR. Internal sensor clock.
 * No MCU, HAL, OS, interrupt, DMA, heap, UART or attitude dependencies.
 * Register snapshots do NOT guarantee delivery of every ODR sample.
 * FIFO acquisition is not enabled here.
 */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ICM_IMU_ADDR_LOW       0x68U
#define ICM_IMU_ADDR_HIGH      0x69U
#define ICM_IMU_WHO_AM_I_REG   0x75U
#define ICM_IMU_ACCEL_LSB_G    2048.0f
#define ICM_IMU_GYRO_LSB_DPS   16.4f

typedef enum {
    ICM_IMU_UNKNOWN = 0,
    ICM_IMU_42670P = 42670,
    ICM_IMU_42688P = 42688
} icm_imu_model_t;

enum {
    ICM_IMU_OK = 0,
    ICM_IMU_ERR_ARGUMENT = -1,
    ICM_IMU_ERR_BUS = -2,
    ICM_IMU_ERR_ID = -3,
    ICM_IMU_ERR_TIMEOUT = -4,
    ICM_IMU_ERR_VERIFY = -5,
    ICM_IMU_ERR_STATE = -6,
    ICM_IMU_ERR_DATA = -7
};

typedef struct {
    void *context;
    /* addr7 is ALWAYS unshifted (0x68/0x69), reg is an 8-bit address.
     * Return 0 only when the ENTIRE transaction is complete; nonzero on error.
     * read_regs sets the register pointer, then reads consecutive bytes.
     * Use repeated START normally. A validated single-controller STOP+START
     * adapter may be used for a particular module. NACK the final read byte.
     * Implement bounded I/O timeouts in the adapter; do not return early for DMA.
     */
    int (*read_regs)(void *context, uint8_t addr7, uint8_t reg,
                     uint8_t *data, uint16_t length);
    int (*write_reg)(void *context, uint8_t addr7, uint8_t reg, uint8_t value);
    /* Must wait AT LEAST the requested microseconds, including 80000 us. */
    void (*delay_us)(void *context, uint32_t duration_us);
} icm_imu_bus_t;

typedef struct {
    int16_t accel_raw[3];             /* X, Y, Z; original signed integers */
    int16_t gyro_raw[3];
    float accel_g[3];                 /* includes gravity; no bias subtraction */
    float gyro_dps[3];                /* degrees/second; no bias subtraction */
} icm_imu_sample_t;

typedef struct {
    icm_imu_bus_t bus;
    icm_imu_model_t model;
    uint16_t odr_hz;
    uint8_t addr7;
    uint8_t initialized;
    int last_bus_error;               /* last nonzero adapter return value */
} icm_imu_t;

/* Read identity without altering configuration. Use a powered, bank-0 device.
 * On failure model is UNKNOWN. addr7 must be 0x68 or 0x69. */
int icm_imu_identify(const icm_imu_bus_t *bus, uint8_t addr7,
                     icm_imu_model_t *model);

/* Probe, soft reset once, verify reset completion, configure and read back.
 * All three bus callbacks are required. FIFO remains bypassed.
 * Assumes power is stable and bank 0 is selected before the first identity read.
 * Reinitialization discards previous configuration. Exclusive device ownership.
 * The bus clock is chosen by the adapter, not by this driver (<=400 kHz profile).
 */
int icm_imu_init(icm_imu_t *device, const icm_imu_bus_t *bus, uint8_t addr7);

/* One 12-byte register burst: accel X/Y/Z followed by gyro X/Y/Z.
 * No data-ready checking, FIFO consumption, timestamp or freshness guarantee.
 * Polling too fast may repeat samples; polling too slowly skips samples.
 * Output is unchanged on error. Not reentrant on the same device/bus.
 */
int icm_imu_read(icm_imu_t *device, icm_imu_sample_t *sample);
/* Decode one complete 12-byte big-endian burst acquired by IRQ transport. */
int icm_imu_decode_motion(const uint8_t *bytes, icm_imu_sample_t *sample);

#ifdef __cplusplus
}
#endif
#endif
