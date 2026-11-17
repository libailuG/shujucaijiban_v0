#include "stm32h7xx.h"
#include "reserved_uart2.h"
#include <string.h>
#include "reserved_uart1.h"
/* JP4: pin1 GND, pin2 PA1 RX, pin3 PA0 TX, pin4 3.3V.
 * UART4 AF8, 8N1; Variable length: mode1 observation or mode20 acquisition frames. */
static uint8_t tx_buffer[sizeof(ControlMode1Monitor)];
volatile uint32_t g_uart1_tx_drop_count;
static volatile uint32_t tx_busy;
static uint32_t tx_index, tx_length;
volatile uint32_t g_uart1_tx_count;
void reserved_uart1_init(void)
{
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;
    RCC->APB1LENR |= RCC_APB1LENR_UART4EN;
    (void)RCC->APB1LENR;
    GPIOA->BSRR = 1UL;
    GPIOA->OTYPER &= ~3UL;
    GPIOA->PUPDR = (GPIOA->PUPDR & ~15UL) | 4UL;
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~15UL) | 10UL;
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~255UL) | 0x88UL;
    GPIOA->MODER = (GPIOA->MODER & ~15UL) | 10UL;
    /* Shared kernel PCLK1=137.5MHz. */
    RCC->D2CCIP2R &= ~RCC_D2CCIP2R_USART28SEL;
    RCC->APB1LRSTR |= RCC_APB1LRSTR_UART4RST;
    RCC->APB1LRSTR &= ~RCC_APB1LRSTR_UART4RST;
    UART4->CR1 = 0U;
    UART4->CR2 = 0U;
    UART4->CR3 = 0U;
    UART4->PRESC = 0U;
    UART4->BRR = (137500000UL + RESERVED_UART1_BAUD / 2U) / RESERVED_UART1_BAUD;
    tx_busy = tx_index = g_uart1_tx_count = 0U;
    g_uart1_tx_drop_count = 0U;
    NVIC_SetPriority(UART4_IRQn, 6U);
    NVIC_ClearPendingIRQ(UART4_IRQn);
    NVIC_EnableIRQ(UART4_IRQn);
    UART4->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}
void reserved_uart1_poll(void)
{
    if (UART4->ISR & USART_ISR_RXNE_RXFNE) (void)UART4->RDR;
    UART4->ICR = USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF;
}
static int send_frame(const void *frame, uint32_t length)
{
    if (!frame || length>sizeof(tx_buffer)) return 0;
    if (tx_busy) { ++g_uart1_tx_drop_count; return 0; }
    memcpy(tx_buffer,frame,length);
    tx_index=0U; tx_length=length; tx_busy=1U;
    __DMB();
    UART4->ICR=USART_ICR_TCCF;
    UART4->CR1 |= USART_CR1_TXEIE_TXFNFIE;
    return 1;
}
int reserved_uart1_send_observation(const ControlMode1Observation *frame)
{
    if (g_uart2_debug_mode!=RESERVED_UART2_CONTROL_MODE1_VALUE ||
        !g_control_mode1_observation_enabled) return 0;
    return send_frame(frame,sizeof(*frame));
}
int reserved_uart1_send_acquisition(const uint8_t frame[ACQUISITION_FRAME_SIZE])
{
    if (g_uart2_debug_mode!=RESERVED_UART2_ACQUISITION_VALUE || !acquisition_mode_is_enabled()) return 0;
    return send_frame(frame,ACQUISITION_FRAME_SIZE);
}
int reserved_uart1_send_control2(const ControlMode2Telemetry *frame)
{
    if (!frame || !((g_uart2_debug_mode==RESERVED_UART2_CONTROL_MODE2_VALUE && frame->version==1U) ||
        (g_uart2_debug_mode==RESERVED_UART2_CONTROL_MODE3_VALUE && frame->version==3U))) return 0;
    return send_frame(frame,sizeof(*frame));
}
void UART4_IRQHandler(void)
{
    if ((UART4->ISR & USART_ISR_TXE_TXFNF) && (UART4->CR1 & USART_CR1_TXEIE_TXFNFIE)) {
        UART4->TDR = tx_buffer[tx_index++];
        if (tx_index >= tx_length)
            UART4->CR1 = (UART4->CR1 & ~USART_CR1_TXEIE_TXFNFIE) | USART_CR1_TCIE;
    }
    if ((UART4->ISR & USART_ISR_TC) && (UART4->CR1 & USART_CR1_TCIE)) {
        UART4->CR1 &= ~USART_CR1_TCIE;
        UART4->ICR = USART_ICR_TCCF;
        ++g_uart1_tx_count;
        __DMB();
        tx_busy = 0U;
    }
    __DSB();
}

