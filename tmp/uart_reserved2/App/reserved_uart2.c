#include "stm32h7xx.h"
#include "imu_bus.h"
#include "reserved_uart2.h"

static const char hello[] = "hello_world\r\n";
static volatile uint32_t tx_busy;
static uint32_t tx_index, previous_send;
volatile uint32_t g_uart2_tx_count;
volatile uint32_t g_uart2_rx_count;
volatile uint32_t g_uart2_last_rx;
volatile uint32_t g_uart2_error_count;

void reserved_uart2_init(void)
{
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;
    RCC->APB2ENR |= RCC_APB2ENR_UART9EN;
    (void)RCC->APB2ENR;
    /* Only PD14/15: PD12/13 remain I2C4 for the IMU. */
    GPIOD->BSRR = 1UL << 15;
    GPIOD->OTYPER &= ~((1UL << 14) | (1UL << 15));
    GPIOD->PUPDR = (GPIOD->PUPDR & ~(15UL << 28)) | (1UL << 28);
    GPIOD->OSPEEDR = (GPIOD->OSPEEDR & ~(15UL << 28)) | (10UL << 28);
    GPIOD->AFR[1] = (GPIOD->AFR[1] & ~(255UL << 24)) | (0xBBUL << 24);
    GPIOD->MODER = (GPIOD->MODER & ~(15UL << 28)) | (10UL << 28);
    /* USART1/6/UART9/USART10 kernel=PCLK2=137.5MHz, prescaler /1.
     * Oversampling16, BRR=149 => 922818.8 baud (+0.132%). */
    RCC->D2CCIP2R &= ~RCC_D2CCIP2R_USART16910SEL;
    RCC->APB2RSTR |= RCC_APB2RSTR_UART9RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_UART9RST;
    UART9->CR1 = 0U;
    UART9->CR2 = 0U; /* One stop bit. */
    UART9->CR3 = 0U; /* No CTS/RTS, FIFO disabled. */
    UART9->PRESC = 0U;
    UART9->BRR = (137500000UL + RESERVED_UART2_BAUD / 2U) / RESERVED_UART2_BAUD;
    tx_busy = tx_index = 0U;
    g_uart2_tx_count = g_uart2_rx_count = g_uart2_last_rx = g_uart2_error_count = 0U;
    NVIC_SetPriority(UART9_IRQn, 6U); /* Lower priority than 1ms IMU/I2C. */
    NVIC_ClearPendingIRQ(UART9_IRQn);
    NVIC_EnableIRQ(UART9_IRQn);
    UART9->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_UE;
    previous_send = imu_time_us();
}

void reserved_uart2_poll(void)
{
    uint32_t now = imu_time_us();
    if (!tx_busy && (uint32_t)(now - previous_send) >= RESERVED_UART2_HELLO_PERIOD_US) {
        previous_send = now; /* Skip missed periods; never send a backlog. */
        tx_index = 0U;
        tx_busy = 1U;
        __DMB();
        UART9->ICR = USART_ICR_TCCF;
        UART9->CR1 |= USART_CR1_TXEIE_TXFNFIE;
    }
}

void UART9_IRQHandler(void)
{
    uint32_t status = UART9->ISR;
    if (status & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)) {
        UART9->ICR = USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF;
        ++g_uart2_error_count;
    }
    if (status & USART_ISR_RXNE_RXFNE) {
        /* Receive is reserved; consume bytes to avoid overrun, no echo. */
        g_uart2_last_rx = UART9->RDR & 0xFFU;
        ++g_uart2_rx_count;
    }
    if ((status & USART_ISR_TXE_TXFNF) && (UART9->CR1 & USART_CR1_TXEIE_TXFNFIE)) {
        UART9->TDR = (uint8_t)hello[tx_index++];
        if (tx_index >= sizeof(hello) - 1U) {
            UART9->CR1 = (UART9->CR1 & ~USART_CR1_TXEIE_TXFNFIE) | USART_CR1_TCIE;
        }
    }
    if ((UART9->ISR & USART_ISR_TC) && (UART9->CR1 & USART_CR1_TCIE)) {
        UART9->CR1 &= ~USART_CR1_TCIE;
        UART9->ICR = USART_ICR_TCCF;
        ++g_uart2_tx_count;
        __DMB();
        tx_busy = 0U;
    }
    __DSB();
}
