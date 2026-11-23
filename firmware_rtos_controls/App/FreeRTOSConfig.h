#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "stm32h7xx.h"

#define configCPU_CLOCK_HZ SystemCoreClock
#define configTICK_RATE_HZ 1000U
#define configUSE_PREEMPTION 1
#define configMAX_PRIORITIES 5
#define configMINIMAL_STACK_SIZE 128U
#define configTICK_TYPE_WIDTH_IN_BITS TICK_TYPE_WIDTH_32_BITS
#define configSUPPORT_STATIC_ALLOCATION 1
#define configSUPPORT_DYNAMIC_ALLOCATION 0
#define configKERNEL_PROVIDED_STATIC_MEMORY 1
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 1
#define configCHECK_FOR_STACK_OVERFLOW 2
#define INCLUDE_vTaskDelay 1
#define INCLUDE_vTaskSuspend 1

/* TIM7, I2C4 and BDMA use priority 5; TIM3 uses 5 for task notification.
 * TIM2 motor DE and TIM6 LED run above the kernel mask and use no RTOS API. */
#define configKERNEL_INTERRUPT_PRIORITY (15U << (8U - __NVIC_PRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY (5U << (8U - __NVIC_PRIO_BITS))

#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

void rtos_phase1_fault(void);
#define configASSERT(condition) do { if (!(condition)) rtos_phase1_fault(); } while (0)

#endif
