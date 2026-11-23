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

#define RED_LED (1UL << 10)

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
    GPIOG->BSRR = RED_LED << 16;
    for (;;) { __NOP(); }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;
    rtos_phase1_fault();
}

void rtos_phase1_imu_ready_from_isr(void)
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

#ifdef ZHUBAN_RTOS_CONTROLS
static void cascade_led_update(void)
{
#ifdef ZHUBAN_RED_LED_TEST
    GPIOG->BSRR = RED_LED << 16; /* Drive PG10 low for the LED5 brightness test. */
#else
    static const uint32_t blink_period_us[] = {1000000U, 300000U, 600000U};
    static uint32_t previous_health = UINT32_MAX;
    static uint32_t cycle_started_us;
    CascadeSnapshot snapshot;

    /* The snapshot's valid mask already excludes missing or stale records. */
    cascade_left_get_snapshot(&snapshot);
    uint32_t left_healthy = snapshot.online && snapshot.valid_mask == 0x7U;
    cascade_right_get_snapshot(&snapshot);
    uint32_t right_healthy = snapshot.online && snapshot.valid_mask == 0x7U;
    uint32_t health = left_healthy | (right_healthy << 1);

    uint32_t now = imu_time_us();
    if (health != previous_health) {
        previous_health = health;
        cycle_started_us = now;
    }

    uint32_t on = 1U;
    if (health != 3U) {
        uint32_t period = blink_period_us[health];
        uint32_t phase = now - cycle_started_us;
        if (phase >= period) {
            phase %= period;
            cycle_started_us = now - phase;
        }
        on = phase < period / 2U;
    }
    GPIOG->BSRR = on ? (RED_LED << 16) : RED_LED;
#endif
}
#endif

static void io_task(void *argument)
{
    (void)argument;
    for (;;) {
        cascade_left_poll();
        cascade_right_poll();
#ifdef ZHUBAN_RTOS_CONTROLS
        cascade_led_update();
#endif
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
