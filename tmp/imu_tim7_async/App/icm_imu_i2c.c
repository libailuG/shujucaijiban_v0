#include "icm_imu_i2c.h"
#include <string.h>

static int address_valid(uint8_t address)
{
    return address == ICM_IMU_ADDR_LOW || address == ICM_IMU_ADDR_HIGH;
}

static int model_valid(icm_imu_model_t model)
{
    return model == ICM_IMU_42670P || model == ICM_IMU_42688P;
}

static icm_imu_model_t model_from_id(uint8_t id)
{
    if (id == 0x67U) return ICM_IMU_42670P;
    if (id == 0x47U) return ICM_IMU_42688P;
    return ICM_IMU_UNKNOWN;
}

static int read_regs(icm_imu_t *d, uint8_t reg, uint8_t *p, uint16_t n)
{
    int r = d->bus.read_regs(d->bus.context, d->addr7, reg, p, n);
    if (r != 0) { d->last_bus_error = r; return ICM_IMU_ERR_BUS; }
    return ICM_IMU_OK;
}

static int write_reg(icm_imu_t *d, uint8_t reg, uint8_t value)
{
    int r = d->bus.write_reg(d->bus.context, d->addr7, reg, value);
    if (r != 0) { d->last_bus_error = r; return ICM_IMU_ERR_BUS; }
    return ICM_IMU_OK;
}

static int update_bits(icm_imu_t *d, uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t old;
    int r = read_regs(d, reg, &old, 1);
    if (r != 0) return r;
    return write_reg(d, reg, (uint8_t)((old & (uint8_t)~mask) | (value & mask)));
}

int icm_imu_identify(const icm_imu_bus_t *bus, uint8_t addr7,
                     icm_imu_model_t *model)
{
    uint8_t id;
    if (model != 0) *model = ICM_IMU_UNKNOWN;
    if (bus == 0 || bus->read_regs == 0 || model == 0 || !address_valid(addr7))
        return ICM_IMU_ERR_ARGUMENT;
    if (bus->read_regs(bus->context, addr7, ICM_IMU_WHO_AM_I_REG, &id, 1) != 0)
        return ICM_IMU_ERR_BUS;
    *model = model_from_id(id);
    return *model == ICM_IMU_UNKNOWN ? ICM_IMU_ERR_ID : ICM_IMU_OK;
}

static int reset_and_wait(icm_imu_t *d)
{
    uint8_t status, id, seen = 0, attempt;
    int r;
    const uint8_t is70 = (uint8_t)(d->model == ICM_IMU_42670P);
    r = write_reg(d, is70 ? 0x02U : 0x11U, is70 ? 0x10U : 0x01U);
    if (r != 0) return r;
    /* 20 ms is a conservative policy, not the datasheet minimum. */
    d->bus.delay_us(d->bus.context, 20000U);
    for (attempt = 0; attempt < 20U; ++attempt) {
        if (is70) r = update_bits(d, 0x36U, 0x0CU, 0x00U);
        else r = write_reg(d, 0x76U, 0x00U);
        if (r == 0 && !seen) {
            r = read_regs(d, is70 ? 0x3AU : 0x2DU, &status, 1);
            if (r == 0 && (status & 0x10U)) seen = 1;
        }
        /* RESET_DONE is read-to-clear: latch it across a failed ID read. */
        if (r == 0 && read_regs(d, 0x75U, &id, 1) == 0 && seen) {
            if (model_from_id(id) != d->model) return ICM_IMU_ERR_ID;
            return ICM_IMU_OK;
        }
        d->bus.delay_us(d->bus.context, 5000U);
    }
    return ICM_IMU_ERR_TIMEOUT;
}

static int configure_42688_interface(icm_imu_t *d)
{
    int r, restore;
    r = write_reg(d, 0x76U, 1U);
    if (r != 0) return r;
    /* DS-000347 I2C interface settings in user bank 1. */
    r = update_bits(d, 0x7CU, 0x13U, 0x10U);
    /* Interface-mode changes can affect the final ACK. Accept an ambiguous
     * write only if a fresh read proves the requested bits actually took effect.
     * A missing device or an unapplied write still fails initialization.
     */
    if (r == ICM_IMU_ERR_BUS) {
        uint8_t verified;
        if (read_regs(d, 0x7CU, &verified, 1) == 0 &&
            (verified & 0x13U) == 0x10U) r = ICM_IMU_OK;
    }
    if (r == 0) r = update_bits(d, 0x7AU, 0x40U, 0x00U);
    restore = write_reg(d, 0x76U, 0U);
    if (r != 0) return r;
    if (restore != 0) return restore;
    r = write_reg(d, 0x13U, 0x09U);
    if (r != 0) return r;
    /* Big-endian sensor data; preserve reserved bits. FIFO is bypassed. */
    r = update_bits(d, 0x4CU, 0xF3U, 0x32U);
    if (r != 0) return r;
    return write_reg(d, 0x16U, 0x00U);
}

int icm_imu_init(icm_imu_t *d, const icm_imu_bus_t *bus, uint8_t addr7)
{
    icm_imu_bus_t copy;
    uint8_t id, cfg[3], interface_value, is70, power_reg, interface_reg;
    int r;
    if (d == 0) return ICM_IMU_ERR_ARGUMENT;
    if (bus == 0 || bus->read_regs == 0 || bus->write_reg == 0 ||
        bus->delay_us == 0 || !address_valid(addr7)) {
        d->initialized = 0;
        return ICM_IMU_ERR_ARGUMENT;
    }
    copy = *bus; /* Also supports passing &device.bus during reinitialization. */
    memset(d, 0, sizeof(*d));
    d->bus = copy;
    d->addr7 = addr7;
    r = read_regs(d, 0x75U, &id, 1);
    if (r != 0) return r;
    d->model = model_from_id(id);
    if (!model_valid(d->model)) return ICM_IMU_ERR_ID;
    is70 = (uint8_t)(d->model == ICM_IMU_42670P);
    d->odr_hz = is70 ? 1600U : 32000U;
    r = reset_and_wait(d);
    if (r != 0) return r;
    if (is70) {
        /* Sensor data big endian; FIFO count setting is irrelevant in bypass. */
        r = update_bits(d, 0x35U, 0x70U, 0x10U);
        if (r == 0) r = update_bits(d, 0x28U, 0x03U, 0x01U);
    } else r = configure_42688_interface(d);
    if (r != 0) return r;
    /* FS=0 gives +/-2000 dps and +/-16 g on both devices.
       42670 uses code 5 (1600 Hz); 42688 uses code 1 (32000 Hz). */
    r = write_reg(d, is70 ? 0x20U : 0x4FU, is70 ? 0x05U : 0x01U);
    if (r != 0) return r;
    r = write_reg(d, is70 ? 0x21U : 0x50U, is70 ? 0x05U : 0x01U);
    if (r != 0) return r;
    power_reg = is70 ? 0x1FU : 0x4EU;
    r = write_reg(d, power_reg, 0x0FU); /* accel LN + gyro LN */
    if (r != 0) return r;
    /* No register writes during OFF->ON settling. Conservative for both. */
    d->bus.delay_us(d->bus.context, 80000U);
    r = read_regs(d, power_reg, cfg, 3);
    if (r != 0) return r;
    if ((cfg[0] & 0x0FU) != 0x0FU || cfg[1] != (is70 ? 0x05U : 0x01U) || cfg[2] != cfg[1])
        return ICM_IMU_ERR_VERIFY;
    interface_reg = is70 ? 0x35U : 0x4CU;
    r = read_regs(d, interface_reg, &interface_value, 1);
    if (r != 0) return r;
    if ((interface_value & 0x10U) == 0U) return ICM_IMU_ERR_VERIFY;
    d->initialized = 1;
    return ICM_IMU_OK;
}

static int16_t signed_be16(const uint8_t *p)
{
    uint16_t u = (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
    int32_t value = u;
    if (u & 0x8000U) value -= 65536L;
    return (int16_t)value;
}

int icm_imu_decode_motion(const uint8_t *bytes, icm_imu_sample_t *sample)
{
    if (bytes == 0 || sample == 0) return ICM_IMU_ERR_ARGUMENT;
    unsigned int axis;
    icm_imu_sample_t result;
    for (axis = 0; axis < 3U; ++axis) {
        result.accel_raw[axis] = signed_be16(bytes + 2U * axis);
        result.gyro_raw[axis] = signed_be16(bytes + 6U + 2U * axis);
        if (result.accel_raw[axis] == (-32767 - 1) ||
            result.gyro_raw[axis] == (-32767 - 1)) return ICM_IMU_ERR_DATA;
        result.accel_g[axis] = result.accel_raw[axis] / ICM_IMU_ACCEL_LSB_G;
        result.gyro_dps[axis] = result.gyro_raw[axis] / ICM_IMU_GYRO_LSB_DPS;
    }
    *sample = result;
    return ICM_IMU_OK;
}

int icm_imu_read(icm_imu_t *d, icm_imu_sample_t *sample)
{
    uint8_t bytes[12];
    int r;
    if (d == 0 || sample == 0) return ICM_IMU_ERR_ARGUMENT;
    if (!d->initialized || !model_valid(d->model) || d->bus.read_regs == 0)
        return ICM_IMU_ERR_STATE;
    r = read_regs(d, d->model == ICM_IMU_42670P ? 0x0BU : 0x1FU, bytes, 12);
    if (r != 0) return r;
    return icm_imu_decode_motion(bytes, sample);
}

