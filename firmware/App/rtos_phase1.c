#include "FreeRTOS.h"
#include "task.h"
#include "rtos_phase1.h"
#include "imu.h"
#include "imu_bus.h"
#include "motor_bus.h"
#include "battery.h"
#include "cascade_left.h"
#include "cascade_right.h"
#include "reserved_uart1.h"
#include "reserved_uart2.h"

static TaskHandle_t motor_task_handle, imu_task_handle;
static StaticTask_t motor_task_storage, imu_task_storage, io_task_storage;
static StackType_t motor_stack[512], imu_stack[512], io_stack[768];
#ifdef ZHUBAN_RTOS_CONTROLS
static StaticTask_t control_task_storage;
static StackType_t control_stack[2048];
#endif
volatile uint32_t g_rtos_motor_coalesced_slots;
volatile uint32_t g_rtos_motor_max_lateness_us;
volatile uint32_t g_rtos_motor_slot_count;

void rtos_phase1_fault(void)
{
    __disable_irq();
    GPIOG->BSRR = 1UL << (11U + 16U); /* Red LED on. */
    for (;;) { __NOP(); }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;
    rtos_phase1_fault();
}

void rtos_phase1_imu_sample_ready_from_isr(void)
{
    BaseType_t wake = pdFALSE;
    vTaskNotifyGiveFromISR(imu_task_handle, &wake);
    portYIELD_FROM_ISR(wake);
}

void TIM3_IRQHandler(void)
{
    BaseType_t wake = pdFALSE;
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR = 0U;
        vTaskNotifyGiveFromISR(motor_task_handle, &wake);
    }
    __DSB();
    portYIELD_FROM_ISR(wake);
}

static void motor_task(void *argument)
{
    (void)argument;
    RCC->APB1LENR |= RCC_APB1LENR_TIM3EN;
    (void)RCC->APB1LENR;
    TIM3->PSC = 274U; /* APB1 timer clock 275 MHz, count at 1 MHz. */
    TIM3->ARR = MOTOR_PERIOD_US / 2U - 1U;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0U;
    NVIC_SetPriority(TIM3_IRQn, 5U);
    NVIC_ClearPendingIRQ(TIM3_IRQn);
    NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->DIER = TIM_DIER_UIE;
    TIM3->CR1 = TIM_CR1_CEN;

    for (;;) {
        /* Tick timeout services RX and DMA aborts between transmission slots. */
        uint32_t slots = ulTaskNotifyTake(pdTRUE, 1U);
        if (slots) {
            ++g_rtos_motor_slot_count;
            if (slots > 1U) g_rtos_motor_coalesced_slots += slots - 1U;
            if (slots == 1U && TIM3->CNT > g_rtos_motor_max_lateness_us)
                g_rtos_motor_max_lateness_us = TIM3->CNT;
        }
        motor_bus_rtos_poll(slots);
    }
}

static void io_task(void *argument)
{
    (void)argument;
    for (;;) {
        cascade_left_poll();
        cascade_right_poll();
        battery_poll();
#ifndef ZHUBAN_RTOS_CONTROLS
        reserved_uart2_poll();
        reserved_uart1_poll();
#endif
        TIM6->ARR = g_battery_data.led_half_period_ms * 10U - 1U;
        vTaskDelay(1U);
    }
}

#ifdef ZHUBAN_RTOS_CONTROLS
static void control_task(void *argument)
{
    (void)argument;
    for (;;) {
        reserved_uart2_poll();
        motor_bus_rtos_publish_command();
        reserved_uart1_poll();
        vTaskDelay(1U);
    }
}
#endif

void rtos_phase1_start(void)
{
    motor_task_handle = xTaskCreateStatic(motor_task, "motor", sizeof(motor_stack) / sizeof(motor_stack[0]),
                                          NULL, 4U, motor_stack, &motor_task_storage);
    imu_task_handle = xTaskCreateStatic(imu_rtos_task, "imu", sizeof(imu_stack) / sizeof(imu_stack[0]),
                                        NULL, 3U, imu_stack, &imu_task_storage);
    TaskHandle_t io_task_handle = xTaskCreateStatic(io_task, "io", sizeof(io_stack) / sizeof(io_stack[0]),
                                                   NULL, 1U, io_stack, &io_task_storage);
    configASSERT(motor_task_handle && imu_task_handle && io_task_handle);
#ifdef ZHUBAN_RTOS_CONTROLS
    TaskHandle_t control_task_handle = xTaskCreateStatic(control_task, "control", sizeof(control_stack) / sizeof(control_stack[0]),
                                                        NULL, 2U, control_stack, &control_task_storage);
    configASSERT(control_task_handle);
#endif
    vTaskStartScheduler();
    rtos_phase1_fault();
}
