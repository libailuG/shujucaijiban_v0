#include "stm32h7xx.h"
#include "imu_bus.h"
#include "battery.h"

volatile BatteryData g_battery_data;
static uint32_t started_at;
static uint32_t active, ready;

static int wait_bits(volatile uint32_t *reg, uint32_t mask, uint32_t value)
{
    uint32_t start = imu_time_us();
    while ((*reg & mask) != value) {
        if ((uint32_t)(imu_time_us() - start) >= 20000U) return 0;
    }
    return 1;
}

static void fail(int32_t error)
{
    ready = active = 0U;
    g_battery_data.valid = 0U;
    g_battery_data.level = 0U;
    g_battery_data.last_error = error;
    g_battery_data.led_half_period_ms = 125U;
}

void battery_init(void)
{
    ready = active = 0U;
    g_battery_data = (BatteryData){0};
    g_battery_data.led_half_period_ms = 125U;
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOFEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_ADC12EN;
    (void)RCC->AHB1ENR;
    GPIOF->MODER |= 3UL << 22; /* PF11 = ADC1_INP2, analog, no pull. */
    GPIOF->PUPDR &= ~(3UL << 22);
    /* ADC1/2 are owned exclusively by this module. HSI64 -> per_ck ->
     * async /4 -> H723 internal /2 = 8MHz ADC conversion clock.
     * BOOST=01 for 6.25..12.5MHz, independent of CPU 550MHz. */
    RCC->D1CCIPR &= ~RCC_D1CCIPR_CKPERSEL;
    RCC->D3CCIPR = (RCC->D3CCIPR & ~RCC_D3CCIPR_ADCSEL) |
        RCC_D3CCIPR_ADCSEL_1;
    RCC->AHB1RSTR |= RCC_AHB1RSTR_ADC12RST;
    RCC->AHB1RSTR &= ~RCC_AHB1RSTR_ADC12RST;
    ADC12_COMMON->CCR = ADC_CCR_PRESC_1;
    ADC1->CR = ADC_CR_ADVREGEN | ADC_CR_BOOST_0;
    imu_delay_us(0, 25U); /* ADC regulator stabilization >=20us. */
    ADC1->DIFSEL_RES12 = 0U;
    ADC1->CR |= ADC_CR_ADCALLIN | ADC_CR_ADCAL;
    if (!wait_bits(&ADC1->CR, ADC_CR_ADCAL, 0U)) { fail(-1); return; }
    imu_delay_us(0, 2U); /* More than four ADC cycles after calibration. */
    ADC1->CFGR = 0U; /* 16-bit, software trigger, single conversion. */
    ADC1->CFGR2 = 0U;
    ADC1->PCSEL_RES0 = ADC_PCSEL_PCSEL_2;
    ADC1->SMPR1 = ADC_SMPR1_SMP2; /* 810.5 cycles for 100k/10k divider. */
    ADC1->SQR1 = 2UL << ADC_SQR1_SQ1_Pos;
    ADC1->ISR = ADC_ISR_ADRDY;
    ADC1->CR |= ADC_CR_ADEN;
    if (!wait_bits(&ADC1->ISR, ADC_ISR_ADRDY, ADC_ISR_ADRDY)) {
        fail(-2); return;
    }
    ready = 1U;
    started_at = imu_time_us() - 10000U;
}

static void publish(uint32_t raw)
{
    static const float boundary[3] = {14.0f, 14.8f, 15.6f};
    static const uint32_t half_period[4] = {125U, 250U, 500U, 1000U};
    float v = (float)raw * (BATTERY_REFERENCE_V * BATTERY_DIVIDER_GAIN *
        BATTERY_CALIBRATION_GAIN / 65535.0f);
    float filtered = g_battery_data.valid ?
        g_battery_data.filtered_voltage_v + (v - g_battery_data.filtered_voltage_v) / 16.0f : v;
    uint32_t level = g_battery_data.level;
    if (!g_battery_data.valid) {
        level = 1U;
        while (level < 4U && filtered >= boundary[level - 1U]) ++level;
    } else {
        /* Drop at nominal boundary; require +0.10V to rise again. */
        while (level > 1U && filtered < boundary[level - 2U]) --level;
        while (level < 4U && filtered >= boundary[level - 1U] + 0.10f) ++level;
    }
    g_battery_data.adc_raw = raw;
    g_battery_data.voltage_v = v;
    g_battery_data.filtered_voltage_v = filtered;
    g_battery_data.level = level;
    g_battery_data.led_half_period_ms = half_period[level - 1U];
    ++g_battery_data.sample_count;
    g_battery_data.last_error = 0;
    g_battery_data.valid = 1U;
}

void battery_poll(void)
{
    if (!ready) return;
    uint32_t now = imu_time_us();
    if (active) {
        if (ADC1->ISR & ADC_ISR_EOC) {
            uint32_t raw = ADC1->DR;
            ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
            active = 0U;
            publish(raw);
        } else if ((uint32_t)(now - started_at) >= 2000U) {
            ADC1->CR |= ADC_CR_ADSTP;
            fail(-3);
        }
    }
    if (ready && !active && (uint32_t)(now - started_at) >= 10000U) {
        started_at = now;
        active = 1U;
        ADC1->CR |= ADC_CR_ADSTART;
    }
}

