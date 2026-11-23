#ifndef RTOS_PHASE1_H
#define RTOS_PHASE1_H
#include <stdint.h>

void rtos_phase1_start(void);
void rtos_phase1_imu_sample_ready_from_isr(void);
extern volatile uint32_t g_rtos_motor_slot_count;
extern volatile uint32_t g_rtos_motor_coalesced_slots;
extern volatile uint32_t g_rtos_motor_max_lateness_us;

#endif
