#include "stm32h7xx.h"

/* Start on HSI; main() raises the clock after GPIO diagnostic setup. */
uint32_t SystemCoreClock = 64000000UL;
uint32_t SystemD2Clock = 64000000UL;

void ExitRun0Mode(void)
{
    /* Board uses internal core LDO with capacitors on VCAP. */
    PWR->CR3 |= PWR_CR3_LDOEN;
    while ((PWR->CSR1 & PWR_CSR1_ACTVOSRDY) == 0U) {}
}

void SystemInit(void)
{
    /* Called by the ST startup before C runtime initializes global variables. */
    SCB->CPACR |= (0xFUL << 20);
    /* Keep D1/D3 debug clocks and sleep debug access enabled for bring-up.
     * This is a development image; low-power consumption is not a goal. */
    DBGMCU->CR |= 0x00600007UL;
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0U) {}
    RCC->CFGR &= ~RCC_CFGR_SW;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {}
    /* HSIDIV is /1 after reset. This firmware is entered through reset. */
    RCC->D1CFGR = 0U;
    RCC->D2CFGR = 0U;
    RCC->D3CFGR = 0U;
    SCB->VTOR = FLASH_BASE;
    __DSB();
    __ISB();
}

void SystemCoreClockUpdate(void)
{
    /* The two clock configurations supported by this application. */
    if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL1) {
        SystemCoreClock = 550000000UL;
        SystemD2Clock = 275000000UL;
    } else {
        SystemCoreClock = 64000000UL;
        SystemD2Clock = SystemCoreClock >>
            (((RCC->D1CFGR & RCC_D1CFGR_HPRE) == RCC_D1CFGR_HPRE_DIV2) ? 1U : 0U);
    }
}

static int wait_bits(volatile uint32_t *reg, uint32_t mask, uint32_t value)
{
    /* Bounded startup wait, executed on HSI before switching the CPU clock. */
    uint32_t timeout = 16000000UL;
    while ((*reg & mask) != value) {
        if (--timeout == 0U) return 0;
    }
    return 1;
}

/* Return 0 on success; errors: 1=BOOST option, 2=VOS, 3=HSE,
 * 4=PLL disable, 5=Flash, 6=PLL lock, 7=SYSCLK switch.
 * Never programs option bytes automatically. Enter only after hardware reset. */
uint32_t BoardClock_Config550MHz(void)
{
    if ((FLASH->OPTSR2_CUR & FLASH_OPTSR2_CPUFREQ_BOOST) == 0U) return 1U;

    /* STM32H723 VOS0 encoding is 00 (different from H743 overdrive). */
    PWR->D3CR &= ~PWR_D3CR_VOS;
    (void)PWR->D3CR;
    if (!wait_bits(&PWR->D3CR, PWR_D3CR_VOSRDY, PWR_D3CR_VOSRDY)) return 2U;

    RCC->CR &= ~RCC_CR_HSEBYP; /* Board has a crystal, not a bypass clock. */
    RCC->CR |= RCC_CR_HSEON;
    if (!wait_bits(&RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY)) return 3U;

    RCC->CR &= ~RCC_CR_PLL1ON;
    if (!wait_bits(&RCC->CR, RCC_CR_PLL1RDY, 0U)) return 4U;
    /* HSE 25MHz / M5 * N110 / P1 = 550MHz; VCO input=5MHz.
     * VCO wide range, integer mode, only P output used. */
    RCC->PLLCKSELR = (RCC->PLLCKSELR &
        ~(RCC_PLLCKSELR_PLLSRC | RCC_PLLCKSELR_DIVM1)) |
        RCC_PLLCKSELR_PLLSRC_HSE | (5UL << RCC_PLLCKSELR_DIVM1_Pos);
    RCC->PLL1DIVR = (109UL << RCC_PLL1DIVR_N1_Pos) |
        (0UL << RCC_PLL1DIVR_P1_Pos) | (3UL << RCC_PLL1DIVR_Q1_Pos) |
        (1UL << RCC_PLL1DIVR_R1_Pos);
    RCC->PLL1FRACR = 0U;
    RCC->PLLCFGR = (RCC->PLLCFGR &
        ~(RCC_PLLCFGR_PLL1VCOSEL | RCC_PLLCFGR_PLL1RGE |
          RCC_PLLCFGR_PLL1FRACEN | RCC_PLLCFGR_DIVP1EN |
          RCC_PLLCFGR_DIVQ1EN | RCC_PLLCFGR_DIVR1EN)) |
        RCC_PLLCFGR_PLL1RGE_2 | RCC_PLLCFGR_DIVP1EN;

    /* Set latency and all bus dividers BEFORE increasing SYSCLK.
     * VOS0 AXI=275MHz: LATENCY=3, WRHIGHFREQ=3 (RM0468). */
    FLASH->ACR = (FLASH->ACR & ~(FLASH_ACR_LATENCY | FLASH_ACR_WRHIGHFREQ)) |
        3UL | FLASH_ACR_WRHIGHFREQ;
    if (!wait_bits(&FLASH->ACR, FLASH_ACR_LATENCY | FLASH_ACR_WRHIGHFREQ,
                   3UL | FLASH_ACR_WRHIGHFREQ)) return 5U;
    RCC->D1CFGR = RCC_D1CFGR_HPRE_DIV2 | RCC_D1CFGR_D1PPRE_DIV2;
    RCC->D2CFGR = RCC_D2CFGR_D2PPRE1_DIV2 | RCC_D2CFGR_D2PPRE2_DIV2;
    RCC->D3CFGR = RCC_D3CFGR_D3PPRE_DIV2;

    RCC->CR |= RCC_CR_PLL1ON;
    if (!wait_bits(&RCC->CR, RCC_CR_PLL1RDY, RCC_CR_PLL1RDY)) return 6U;
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL1;
    if (!wait_bits(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL1)) return 7U;
    __DSB();
    __ISB();
    SystemCoreClockUpdate();
    return 0U;
}
