#ifndef X1_ASSIST_OSCILLATION_MONITOR_H
#define X1_ASSIST_OSCILLATION_MONITOR_H

#include <stdint.h>

typedef struct {
    float positions[150][2];
    float torque_abs[150];
    uint8_t reversals[150][2];
    float extreme[2];
    int8_t direction[2];
    float frequency_hz[2];
    uint32_t sample_counter;
    uint32_t last_reversal_sample[2];
    uint8_t reversal_seen[2];
    uint16_t recovery_samples;
    uint16_t count;
    uint16_t index;
    uint8_t latched;
} AssistOscillationMonitor_t;

void AssistOscillationMonitor_Init(AssistOscillationMonitor_t *monitor);
void AssistOscillationMonitor_Reset(AssistOscillationMonitor_t *monitor);
uint8_t AssistOscillationMonitor_Update(
    AssistOscillationMonitor_t *monitor,
    const float position_rad[2],
    const float velocity_rad_s[2],
    float commanded_torque_nm,
    float *left_torque_nm,
    float *right_torque_nm);

#endif
