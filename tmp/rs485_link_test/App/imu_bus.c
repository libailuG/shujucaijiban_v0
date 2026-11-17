#include "stm32h7xx.h"
#include "imu_bus.h"

#define SCL (1UL << 12)
#define SDA (1UL << 13)
#define PINS (SCL | SDA)
#define ERROR_FLAGS (I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR)

/* One transaction plus one completion slot. No backlog is accumulated.
 * I2C4 EV/ER and TIM7 use the same priority, so their handlers cannot
 * preempt each other. Main accesses the slot under a short IRQ mask. */
static volatile uint32_t sample_active, sample_done;
static volatile uint32_t sample_started;
static volatile int sample_result;
static uint8_t sample_bytes[12];
static uint8_t sample_address, sample_phase, sample_index;

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
            if ((uint32_t)(imu_time_us() - start) >= 1000U) return -10;
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
    int result = ((GPIOD->IDR & PINS) == PINS) ? 0 : -12;
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
    (void)imu_bus_recover();
    NVIC_SetPriority(I2C4_EV_IRQn, 5U);
    NVIC_SetPriority(I2C4_ER_IRQn, 5U);
    NVIC_ClearPendingIRQ(I2C4_EV_IRQn);
    NVIC_ClearPendingIRQ(I2C4_ER_IRQn);
    NVIC_EnableIRQ(I2C4_EV_IRQn);
    NVIC_EnableIRQ(I2C4_ER_IRQn);
}

static void sample_complete(int result)
{
    /* Error path aborts through PE rather than waiting for a STOP in IRQ.
     * Physical bus recovery, if needed, runs later in the main loop. */
    I2C4->CR1 = result ? 0U : I2C_CR1_PE;
    I2C4->CR2 = 0U;
    I2C4->ICR = 0x3F38U;
    I2C4->CR1 = I2C_CR1_PE;
    sample_result = result;
    sample_active = 0U;
    __DMB();
    sample_done = 1U;
}

void imu_bus_schedule_sample(uint8_t address)
{
    if (sample_active) {
        /* Checked every 1ms. Bound a stalled transfer without an IRQ wait. */
        if ((uint32_t)(imu_time_us() - sample_started) >= 800U)
            sample_complete(-10);
        return;
    }
    if (sample_done) return; /* Main has not consumed the previous result. */
    sample_started = imu_time_us();
    if ((I2C4->ISR & I2C_ISR_BUSY) != 0U) { sample_complete(-12); return; }
    sample_address = address;
    sample_phase = 0U;
    sample_index = 0U;
    sample_active = 1U;
    I2C4->ICR = 0x3F38U;
    I2C4->CR1 = I2C_CR1_PE | I2C_CR1_TXIE | I2C_CR1_TCIE |
        I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_ERRIE;
    I2C4->CR2 = ((uint32_t)address << 1) |
        (1UL << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
}

void I2C4_EV_IRQHandler(void)
{
    uint32_t flags = I2C4->ISR;
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
            I2C4->CR1 = I2C_CR1_PE | I2C_CR1_RXIE | I2C_CR1_STOPIE |
                I2C_CR1_NACKIE | I2C_CR1_ERRIE;
            I2C4->CR2 = ((uint32_t)sample_address << 1) |
                (12UL << I2C_CR2_NBYTES_Pos) | I2C_CR2_RD_WRN |
                I2C_CR2_AUTOEND | I2C_CR2_START; /* Repeated START. */
        }
        if ((flags & I2C_ISR_RXNE) && sample_phase == 2U) {
            uint8_t byte = (uint8_t)I2C4->RXDR;
            if (sample_index < 12U) sample_bytes[sample_index++] = byte;
        }
        /* RXNE may coexist with STOPF for the final byte: consume it first. */
        if (flags & I2C_ISR_STOPF)
            sample_complete((sample_phase == 2U && sample_index == 12U) ? 0 : -12);
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
    I2C4->CR1 = 0U;
    I2C4->CR2 = 0U;
    I2C4->ICR = 0x3F38U;
    sample_active = sample_done = 0U;
    I2C4->CR1 = I2C_CR1_PE;
    __set_PRIMASK(saved);
}

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

static int begin(uint8_t address)
{
    if (address != 0x68U && address != 0x69U) return -13;
    if ((I2C4->ISR & I2C_ISR_BUSY) != 0U) return -12;
    I2C4->ICR = 0x3F38U;
    return 0;
}

int imu_bus_read(void *ctx, uint8_t address, uint8_t reg, uint8_t *data, uint16_t size)
{
    (void)ctx;
    if (!data || size == 0U || size > 255U) return -13;
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
}

int imu_bus_write(void *ctx, uint8_t address, uint8_t reg, uint8_t value)
{
    (void)ctx;
    int r = begin(address);
    if (r != 0) return r;
    uint32_t start = imu_time_us();
    I2C4->CR2 = ((uint32_t)address << 1) | (2UL << I2C_CR2_NBYTES_Pos) |
        I2C_CR2_AUTOEND | I2C_CR2_START;
    r = wait_flag(I2C_ISR_TXIS, start);
    if (r == 0) { I2C4->TXDR = reg; r = wait_flag(I2C_ISR_TXIS, start); }
    if (r == 0) { I2C4->TXDR = value; r = wait_flag(I2C_ISR_STOPF, start); }
    return finish(r);
}
