#include "stm32h7xx.h"
#include "imu_bus.h"
#ifdef ZHUBAN_RTOS_PHASE1
#include "FreeRTOS.h"
#include "task.h"
#include "rtos_phase1.h"
#endif

#define SCL (1UL << 12)
#define SDA (1UL << 13)
#define PINS (SCL | SDA)
#define ERROR_FLAGS (I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR)

/* One transaction plus one completion slot. No backlog is accumulated.
 * I2C4 EV/ER, BDMA and TIM7 use the same priority, so their handlers cannot
 * preempt each other. Main accesses the slot under a short IRQ mask. */
static volatile uint32_t sample_active, sample_done;
static volatile uint32_t sample_started;
static volatile int sample_result;
/* BDMA channel0 / DMAMUX2 request13 (I2C4_RX). SRAM4, one isolated cache line. */
static uint8_t sample_bytes[32] __attribute__((section(".imu_bdma_rx"), aligned(32)));
static uint8_t sample_address, sample_phase, sample_stop, sample_dma_done;

#ifdef ZHUBAN_RTOS_PHASE1
typedef enum {
    REGISTER_IDLE, REGISTER_SUBADDRESS, REGISTER_WRITE_VALUE,
    REGISTER_RESTART, REGISTER_READ, REGISTER_STOP
} RegisterPhase;

/* The IMU task issues register transfers only while streaming is stopped. */
static volatile RegisterPhase register_phase;
static volatile int register_result;
static uint8_t register_address, register_subaddress, register_value;
static uint8_t *register_rx;
static uint16_t register_remaining;
#define REGISTER_END_IRQS (I2C_CR1_PE | I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_ERRIE)
#endif

static void sample_dma_stop(void)
{
    I2C4->CR1 &= ~I2C_CR1_RXDMAEN;
    BDMA_Channel0->CCR = 0U;
    __DSB();
    BDMA->IFCR = BDMA_IFCR_CGIF0;
    NVIC_ClearPendingIRQ(BDMA_Channel0_IRQn);
}

uint32_t imu_time_us(void) { return TIM2->CNT; }
void imu_delay_us(void *ctx, uint32_t us)
{
    uint32_t start = imu_time_us();
    (void)ctx;
    while ((uint32_t)(imu_time_us() - start) < us) {}
}

static void peripheral_enable(void)
{
    I2C4->CR1 = 0U;
    /* HSI=64MHz, PRESC=3, SCLDEL=7, SDADEL=2, SCLH=15, SCLL=23.
     * Nominal SCL is below 400kHz including analog filter/synchronizers.
     * External pull-ups to 3.3V are required for fast-mode rise times. */
    I2C4->TIMINGR = 0x30720F17UL;
    I2C4->ICR = 0x3F38U;
    I2C4->CR1 = I2C_CR1_PE;
}

int imu_bus_recover(void)
{
    int result;
    I2C4->CR1 = 0U;
    GPIOD->BSRR = PINS;
    GPIOD->OTYPER |= PINS;
    GPIOD->MODER = (GPIOD->MODER & ~((3UL << 24) | (3UL << 26))) |
        (1UL << 24) | (1UL << 26);
    for (unsigned i = 0; i < 9U && (GPIOD->IDR & SDA) == 0U; ++i) {
        GPIOD->BSRR = SCL << 16;
        imu_delay_us(0, 5U);
        GPIOD->BSRR = SCL;
        uint32_t start = imu_time_us();
        while ((GPIOD->IDR & SCL) == 0U) {
            if ((uint32_t)(imu_time_us() - start) >= 1000U) {
                result = -10;
                goto restore_bus;
            }
#ifdef ZHUBAN_RTOS_PHASE1
            vTaskDelay(1U);
#endif
        }
        imu_delay_us(0, 5U);
    }
    /* STOP: SDA low, release SCL, then release SDA. Never drive high. */
    GPIOD->BSRR = (SCL | SDA) << 16;
    imu_delay_us(0, 5U);
    GPIOD->BSRR = SCL;
    imu_delay_us(0, 5U);
    GPIOD->BSRR = SDA;
    imu_delay_us(0, 5U);
    result = ((GPIOD->IDR & PINS) == PINS) ? 0 : -12;
restore_bus:
    GPIOD->AFR[1] = (GPIOD->AFR[1] & ~((15UL << 16) | (15UL << 20))) |
        (4UL << 16) | (4UL << 20);
    GPIOD->MODER = (GPIOD->MODER & ~((3UL << 24) | (3UL << 26))) |
        (2UL << 24) | (2UL << 26);
    peripheral_enable();
    return result;
}

void imu_bus_init(void)
{
    /* Same APB1 timer clock as TIM6: 275MHz, TIM2 is a 32-bit 1MHz clock. */
    RCC->APB1LENR |= RCC_APB1LENR_TIM2EN;
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;
    RCC->AHB4ENR |= RCC_AHB4ENR_BDMAEN; /* SRAM4 has no AHB4 run-clock gate on H723. */
    (void)RCC->AHB4ENR;
    BDMA_Channel0->CCR = 0U;
    BDMA->IFCR = BDMA_IFCR_CGIF0;
    DMAMUX2_Channel0->CCR = 13U;
    BDMA_Channel0->CPAR = (uint32_t)&I2C4->RXDR;
    BDMA_Channel0->CM0AR = (uint32_t)sample_bytes;
    NVIC_SetPriority(BDMA_Channel0_IRQn, 5U);
    NVIC_ClearPendingIRQ(BDMA_Channel0_IRQn);
    NVIC_EnableIRQ(BDMA_Channel0_IRQn);
    RCC->APB4ENR |= RCC_APB4ENR_I2C4EN;
    (void)RCC->APB4ENR;
    TIM2->CR1 = 0U;
    TIM2->PSC = 274U;
    TIM2->ARR = 0xFFFFFFFFUL;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0U;
    TIM2->CNT = 0U;
    TIM2->CR1 = TIM_CR1_CEN;
    /* I2C4 kernel source=HSI, independent of APB4 frequency. */
    RCC->D3CCIPR = (RCC->D3CCIPR & ~RCC_D3CCIPR_I2C4SEL) |
        (2UL << RCC_D3CCIPR_I2C4SEL_Pos);
    GPIOD->PUPDR = (GPIOD->PUPDR & ~((3UL << 24) | (3UL << 26))) |
        (1UL << 24) | (1UL << 26);
    GPIOD->OSPEEDR |= (3UL << 24) | (3UL << 26);
    peripheral_enable();
#ifndef ZHUBAN_RTOS_PHASE1
    (void)imu_bus_recover();
#endif
    NVIC_SetPriority(I2C4_EV_IRQn, 5U);
    NVIC_SetPriority(I2C4_ER_IRQn, 5U);
    NVIC_ClearPendingIRQ(I2C4_EV_IRQn);
    NVIC_ClearPendingIRQ(I2C4_ER_IRQn);
    NVIC_EnableIRQ(I2C4_EV_IRQn);
    NVIC_EnableIRQ(I2C4_ER_IRQn);
}

static void finish_transfer(int result)
{
    /* Abort through PE without waiting for STOP. */
    I2C4->CR1 = result ? 0U : I2C_CR1_PE;
    I2C4->CR2 = 0U;
    I2C4->ICR = 0x3F38U;
    I2C4->CR1 = I2C_CR1_PE;
}

static void sample_complete(int result)
{
    sample_dma_stop();
    finish_transfer(result);
    sample_result = result;
    sample_active = 0U;
    __DMB();
    sample_done = 1U;
#ifdef ZHUBAN_RTOS_PHASE1
    rtos_phase1_imu_ready_from_isr();
#endif
}

static void sample_try_complete(void)
{
    /* STOP and the final DMA write may be observed in either IRQ order.
     * Publish only after both, and reject short transfers / I2C errors. */
    if (sample_active && sample_stop && sample_dma_done) {
        uint32_t errors = I2C4->ISR & ERROR_FLAGS;
        sample_complete((errors || BDMA_Channel0->CNDTR != 0U) ? -12 : 0);
    }
}

void BDMA_Channel0_IRQHandler(void)
{
    uint32_t flags = BDMA->ISR;
    BDMA->IFCR = BDMA_IFCR_CGIF0;
    if (sample_active && sample_phase == 2U) {
        if (flags & BDMA_ISR_TEIF0) sample_complete(-12);
        else if (flags & BDMA_ISR_TCIF0) {
            sample_dma_done = 1U;
            sample_try_complete();
        }
    }
    __DSB();
}

void imu_bus_service_sample(uint8_t address, int sample_due)
{
    if (sample_active) {
        /* Checked every 1ms. Bound a stalled transfer without an IRQ wait. */
        if ((uint32_t)(imu_time_us() - sample_started) >= 800U)
            sample_complete(-10);
        return;
    }
    if (!sample_due || sample_done) return;
    sample_started = imu_time_us();
    if ((I2C4->ISR & I2C_ISR_BUSY) != 0U) { sample_complete(-12); return; }
    sample_address = address;
    sample_phase = 0U;
    sample_stop = sample_dma_done = 0U;
    sample_dma_stop();
    if (SCB->CCR & SCB_CCR_DC_Msk)
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *)sample_bytes, sizeof(sample_bytes));
    __DSB();
    sample_active = 1U;
    I2C4->ICR = 0x3F38U;
    I2C4->CR1 = I2C_CR1_PE | I2C_CR1_TXIE | I2C_CR1_TCIE |
        I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_ERRIE;
    I2C4->CR2 = ((uint32_t)address << 1) |
        (1UL << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
}

#ifdef ZHUBAN_RTOS_PHASE1
static void register_complete(int result)
{
    finish_transfer(result);
    register_result = result;
    register_phase = REGISTER_IDLE;
}

static void register_irq(uint32_t flags)
{
    if (flags & ERROR_FLAGS) {
        register_complete((flags & I2C_ISR_NACKF) ? -11 : -12);
        rtos_phase1_imu_ready_from_isr();
        return;
    }
    if (flags & I2C_ISR_TXIS) {
        if (register_phase == REGISTER_SUBADDRESS) {
            I2C4->TXDR = register_subaddress;
            register_phase = register_rx ? REGISTER_RESTART : REGISTER_WRITE_VALUE;
        } else if (register_phase == REGISTER_WRITE_VALUE) {
            I2C4->TXDR = register_value;
            register_phase = REGISTER_STOP;
        }
    }
    if ((flags & I2C_ISR_TC) && register_phase == REGISTER_RESTART) {
        register_phase = REGISTER_READ;
        I2C4->CR1 = REGISTER_END_IRQS | I2C_CR1_RXIE;
        I2C4->CR2 = ((uint32_t)register_address << 1) |
            ((uint32_t)register_remaining << I2C_CR2_NBYTES_Pos) |
            I2C_CR2_RD_WRN | I2C_CR2_AUTOEND | I2C_CR2_START;
    }
    if ((flags & I2C_ISR_RXNE) && register_phase == REGISTER_READ) {
        *register_rx++ = (uint8_t)I2C4->RXDR;
        if (--register_remaining == 0U) register_phase = REGISTER_STOP;
    }
    if (flags & I2C_ISR_STOPF) {
        register_complete(register_phase == REGISTER_STOP ? 0 : -12);
        rtos_phase1_imu_ready_from_isr();
    }
}
#endif

void I2C4_EV_IRQHandler(void)
{
    uint32_t flags = I2C4->ISR;
#ifdef ZHUBAN_RTOS_PHASE1
    if (register_phase != REGISTER_IDLE) { register_irq(flags); __DSB(); return; }
#endif
    if (!sample_active) { I2C4->ICR = 0x3F38U; __DSB(); return; }
    if (flags & ERROR_FLAGS) {
        sample_complete((flags & I2C_ISR_NACKF) ? -11 : -12);
    } else {
        if ((flags & I2C_ISR_TXIS) && sample_phase == 0U) {
            I2C4->TXDR = 0x1FU; /* ICM-42688-P ACCEL_DATA_X1, bank 0. */
            sample_phase = 1U;
        }
        if ((flags & I2C_ISR_TC) && sample_phase == 1U) {
            sample_phase = 2U;
            BDMA_Channel0->CNDTR = 12U;
            BDMA_Channel0->CCR = BDMA_CCR_MINC | BDMA_CCR_TCIE | BDMA_CCR_TEIE | BDMA_CCR_EN;
            I2C4->CR1 = I2C_CR1_PE | I2C_CR1_RXDMAEN | I2C_CR1_STOPIE |
                I2C_CR1_NACKIE | I2C_CR1_ERRIE;
            I2C4->CR2 = ((uint32_t)sample_address << 1) |
                (12UL << I2C_CR2_NBYTES_Pos) | I2C_CR2_RD_WRN |
                I2C_CR2_AUTOEND | I2C_CR2_START; /* Repeated START. */
        }
        if (flags & I2C_ISR_STOPF) {
            I2C4->ICR = I2C_ICR_STOPCF;
            if (sample_phase != 2U) sample_complete(-12);
            else { sample_stop = 1U; sample_try_complete(); }
        }
    }
    __DSB();
}

void I2C4_ER_IRQHandler(void) { I2C4_EV_IRQHandler(); }

int imu_bus_take_sample(uint8_t data[12], uint32_t *timestamp_us, int *result)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    if (!sample_done) { __set_PRIMASK(saved); return 0; }
    *result = sample_result;
    *timestamp_us = sample_started;
    if (*result == 0) {
        if (SCB->CCR & SCB_CCR_DC_Msk)
            SCB_InvalidateDCache_by_Addr((uint32_t *)sample_bytes, sizeof(sample_bytes));
        __DSB();
        for (unsigned i = 0; i < 12U; ++i) data[i] = sample_bytes[i];
    }
    __DMB();
    sample_done = 0U;
    __set_PRIMASK(saved);
    return 1;
}

void imu_bus_cancel_sample(void)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    sample_dma_stop();
    I2C4->CR1 = 0U;
    I2C4->CR2 = 0U;
    I2C4->ICR = 0x3F38U;
    sample_active = sample_done = 0U;
    I2C4->CR1 = I2C_CR1_PE;
    __set_PRIMASK(saved);
}

static int begin(uint8_t address)
{
    if (address != 0x68U && address != 0x69U) return -13;
    if ((I2C4->ISR & I2C_ISR_BUSY) != 0U) return -12;
    I2C4->ICR = 0x3F38U;
    return 0;
}

#ifdef ZHUBAN_RTOS_PHASE1
static int register_transfer(uint8_t address, uint8_t reg, uint8_t value,
                             uint8_t *data, uint16_t size)
{
    int result = begin(address);
    if (result != 0) return result;
    (void)ulTaskNotifyTake(pdTRUE, 0U);
    register_address = address;
    register_subaddress = reg;
    register_value = value;
    register_rx = data;
    register_remaining = size;
    register_phase = REGISTER_SUBADDRESS;
    I2C4->CR1 = REGISTER_END_IRQS | I2C_CR1_TXIE | (data ? I2C_CR1_TCIE : 0U);
    I2C4->CR2 = ((uint32_t)address << 1) | I2C_CR2_START |
        (data ? (1UL << I2C_CR2_NBYTES_Pos) :
                ((2UL << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND));

    /* Allow a full millisecond even when the first tick is about to fire. */
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1U) + 1U);
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    if (register_phase != REGISTER_IDLE) register_complete(-10);
    result = register_result;
    __set_PRIMASK(saved);
    return result;
}
#else
static int wait_flag(uint32_t flag, uint32_t start)
{
    for (;;) {
        uint32_t status = I2C4->ISR;
        if ((status & ERROR_FLAGS) != 0U)
            return (status & I2C_ISR_NACKF) ? -11 : -12;
        if ((status & flag) != 0U) return 0;
        if ((uint32_t)(imu_time_us() - start) >= 800U) return -10;
    }
}

static int finish(int result)
{
    if (result != 0) {
        if ((I2C4->ISR & I2C_ISR_BUSY) && !(I2C4->ISR & I2C_ISR_ARLO))
            I2C4->CR2 |= I2C_CR2_STOP;
        imu_delay_us(0, 10U);
        peripheral_enable();
    } else I2C4->ICR = I2C_ICR_STOPCF;
    return result;
}

#endif

int imu_bus_read(void *ctx, uint8_t address, uint8_t reg, uint8_t *data, uint16_t size)
{
    (void)ctx;
    if (!data || size == 0U || size > 255U) return -13;
#ifdef ZHUBAN_RTOS_PHASE1
    return register_transfer(address, reg, 0U, data, size);
#else
    int r = begin(address);
    if (r != 0) return r;
    uint32_t start = imu_time_us();
    I2C4->CR2 = ((uint32_t)address << 1) | (1UL << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
    r = wait_flag(I2C_ISR_TXIS, start);
    if (r == 0) { I2C4->TXDR = reg; r = wait_flag(I2C_ISR_TC, start); }
    if (r != 0) return finish(r);
    I2C4->CR2 = ((uint32_t)address << 1) | ((uint32_t)size << I2C_CR2_NBYTES_Pos) |
        I2C_CR2_RD_WRN | I2C_CR2_AUTOEND | I2C_CR2_START;
    for (uint16_t i = 0; i < size; ++i) {
        r = wait_flag(I2C_ISR_RXNE, start);
        if (r != 0) return finish(r);
        data[i] = (uint8_t)I2C4->RXDR;
    }
    return finish(wait_flag(I2C_ISR_STOPF, start));
#endif
}

int imu_bus_write(void *ctx, uint8_t address, uint8_t reg, uint8_t value)
{
    (void)ctx;
#ifdef ZHUBAN_RTOS_PHASE1
    return register_transfer(address, reg, value, 0, 0U);
#else
    int r = begin(address);
    if (r != 0) return r;
    uint32_t start = imu_time_us();
    I2C4->CR2 = ((uint32_t)address << 1) | (2UL << I2C_CR2_NBYTES_Pos) |
        I2C_CR2_AUTOEND | I2C_CR2_START;
    r = wait_flag(I2C_ISR_TXIS, start);
    if (r == 0) { I2C4->TXDR = reg; r = wait_flag(I2C_ISR_TXIS, start); }
    if (r == 0) { I2C4->TXDR = value; r = wait_flag(I2C_ISR_STOPF, start); }
    return finish(r);
#endif
}
