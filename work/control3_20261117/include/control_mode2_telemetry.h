#ifndef CONTROL_MODE2_TELEMETRY_H
#define CONTROL_MODE2_TELEMETRY_H
#include <stdint.h>
/* UART4/JP4, 921600 8N1, LE. Model coordinates [left,right].
 * Version 3: mode3; output_scale=amplitude Nm, average_compute_ms=period s.
 * Version 1 mode2 is unchanged. Version 3 status uses CONTROL3 fault bits.
 * flags: bit0 fresh feedback, bit1 computed torque valid, bit2 permission,
 * bit3 driver command valid, bit4 internal guard, bit5 external suppression. */
#define CONTROL2_TELEMETRY_START 0x324D43AAUL
#define CONTROL2_TELEMETRY_END 0xEE324D43UL
typedef struct {
    uint32_t start, version, size, sequence, timestamp_us;
    int32_t status;
    uint32_t flags, compute_us, period_us;
    float angle_rad[2], speed_rad_s[2], calculated_torque_nm[2];
    float driver_torque_nm[2], output_scale, average_compute_ms;
    uint32_t end;
} ControlMode2Telemetry;
typedef char Control2TelemetrySize[(sizeof(ControlMode2Telemetry)==80U)?1:-1];
#endif
