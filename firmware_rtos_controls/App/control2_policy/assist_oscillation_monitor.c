#include "assist_oscillation_monitor.h"

#include <math.h>
#include <string.h>

/* 100 Hz monitor: 1.5 s window for the observed 1.5 Hz gap oscillation. */
enum {
    MONITOR_WINDOW = 150,
    MONITOR_MIN_REVERSALS = 1,
    MONITOR_MAX_REVERSALS = 7
};

static const float REVERSAL_TRAVEL_RAD = 0.008726646259971648f; /* 0.5 deg */
static const float MIN_OSCILLATION_AMPLITUDE_RAD = 0.20943951023931956f; /* 12 deg */
static const float MAX_OSCILLATION_AMPLITUDE_RAD = 0.4363323129985824f; /* 25 deg */
static const float MIN_OSCILLATION_HZ = 1.20f;
static const float MAX_OSCILLATION_HZ = 1.90f;
static const float MIN_TORQUE_NM = 2.0f;

void AssistOscillationMonitor_Init(AssistOscillationMonitor_t *monitor)
{
    if (monitor == NULL) return;
    memset(monitor, 0, sizeof(*monitor));
}

void AssistOscillationMonitor_Reset(AssistOscillationMonitor_t *monitor)
{
    AssistOscillationMonitor_Init(monitor);
}

static float range_for_side(const AssistOscillationMonitor_t *m, unsigned int side)
{
    float lo = m->positions[0][side];
    float hi = lo;
    for (unsigned int i = 1U; i < MONITOR_WINDOW; ++i) {
        lo = fminf(lo, m->positions[i][side]);
        hi = fmaxf(hi, m->positions[i][side]);
    }
    return hi - lo;
}

static unsigned int reversals_for_side(const AssistOscillationMonitor_t *m,
                                       unsigned int side)
{
    unsigned int total = 0U;
    for (unsigned int i = 0U; i < MONITOR_WINDOW; ++i)
        total += m->reversals[i][side] != 0U ? 1U : 0U;
    return total;
}

uint8_t AssistOscillationMonitor_Update(
    AssistOscillationMonitor_t *m,
    const float position_rad[2],
    const float velocity_rad_s[2],
    float commanded_torque_nm,
    float *left_torque_nm,
    float *right_torque_nm)
{
    if (m == NULL || position_rad == NULL || velocity_rad_s == NULL ||
        left_torque_nm == NULL || right_torque_nm == NULL ||
        !isfinite(position_rad[0]) || !isfinite(position_rad[1]) ||
        !isfinite(velocity_rad_s[0]) || !isfinite(velocity_rad_s[1]) ||
        !isfinite(commanded_torque_nm)) return 0U;

    /* Fixed 0.2 s suppression, then automatic re-arm. */
    if (m->latched != 0U) {
        if (m->recovery_samples < 20U) ++m->recovery_samples;
        if (m->recovery_samples >= 20U) {
            AssistOscillationMonitor_Reset(m);
            *left_torque_nm = 0.0f;
            *right_torque_nm = 0.0f;
            return 0U;
        }
        *left_torque_nm = 0.0f;
        *right_torque_nm = 0.0f;
        return 1U;
    }

    if (m->count == 0U) {
        m->extreme[0] = position_rad[0];
        m->extreme[1] = position_rad[1];
    }
    const unsigned int index = m->index;
    for (unsigned int side = 0U; side < 2U; ++side) {
        const float delta = position_rad[side] - m->extreme[side];
        const uint8_t up = m->direction[side] <= 0 && delta >= REVERSAL_TRAVEL_RAD;
        const uint8_t down = m->direction[side] >= 0 && delta <= -REVERSAL_TRAVEL_RAD;
        m->reversals[index][side] = (uint8_t)(
            (up && m->direction[side] < 0) ||
            (down && m->direction[side] > 0));
        if (m->reversals[index][side] != 0U) {
            if (m->reversal_seen[side] != 0U) {
                const uint32_t interval =
                    m->sample_counter - m->last_reversal_sample[side];
                if (interval != 0U)
                    m->frequency_hz[side] = 50.0f / (float)interval;
            }
            m->last_reversal_sample[side] = m->sample_counter;
            m->reversal_seen[side] = 1U;
        }
        if (up) m->direction[side] = 1;
        else if (down) m->direction[side] = -1;
        if (up || down) m->extreme[side] = position_rad[side];
        else if (m->direction[side] > 0) m->extreme[side] = fmaxf(m->extreme[side], position_rad[side]);
        else if (m->direction[side] < 0) m->extreme[side] = fminf(m->extreme[side], position_rad[side]);
        m->positions[index][side] = position_rad[side];
    }
    (void)velocity_rad_s;
    m->torque_abs[index] = fabsf(commanded_torque_nm);
    ++m->sample_counter;
    if (m->count < MONITOR_WINDOW) ++m->count;
    m->index = (uint16_t)((index + 1U) % MONITOR_WINDOW);

    const float left_range = range_for_side(m, 0U);
    const float right_range = range_for_side(m, 1U);
    const unsigned int left_reversals = reversals_for_side(m, 0U);
    const unsigned int right_reversals = reversals_for_side(m, 1U);
    const float left_hz = m->frequency_hz[0];
    const float right_hz = m->frequency_hz[1];
    float peak_torque = 0.0f;
    for (unsigned int i = 0U; i < MONITOR_WINDOW; ++i)
        peak_torque = fmaxf(peak_torque, m->torque_abs[i]);

    /* Either side can trip the shared shutdown. */
    const uint8_t left_detect = (uint8_t)(
        left_range >= MIN_OSCILLATION_AMPLITUDE_RAD &&
        left_range <= MAX_OSCILLATION_AMPLITUDE_RAD &&
        left_reversals >= MONITOR_MIN_REVERSALS &&
        left_reversals <= MONITOR_MAX_REVERSALS &&
        left_hz >= MIN_OSCILLATION_HZ && left_hz <= MAX_OSCILLATION_HZ);
    const uint8_t right_detect = (uint8_t)(
        right_range >= MIN_OSCILLATION_AMPLITUDE_RAD &&
        right_range <= MAX_OSCILLATION_AMPLITUDE_RAD &&
        right_reversals >= MONITOR_MIN_REVERSALS &&
        right_reversals <= MONITOR_MAX_REVERSALS &&
        right_hz >= MIN_OSCILLATION_HZ && right_hz <= MAX_OSCILLATION_HZ);
    const uint8_t detect = (uint8_t)(m->count == MONITOR_WINDOW &&
        (left_detect != 0U || right_detect != 0U) &&
        peak_torque >= MIN_TORQUE_NM);
    if (detect != 0U) {
        m->latched = 1U;
        m->recovery_samples = 0U;
    }
    if (m->latched != 0U) {
        *left_torque_nm = 0.0f;
        *right_torque_nm = 0.0f;
    }
    return m->latched;
}
