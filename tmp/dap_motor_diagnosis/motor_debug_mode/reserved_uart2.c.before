#include "stm32h7xx.h"
#include "imu_bus.h"
#include "reserved_uart2.h"
#include "mcu_uart_stream.h"
#include "imu.h"
#include "battery.h"

#define RX_RING_SIZE 1024U
#define RX_RING_MASK (RX_RING_SIZE - 1U)
static uint8_t tx_buffer[TD_MCU_FRAME_SIZE];
static uint8_t rx_ring[RX_RING_SIZE];
static volatile uint32_t rx_head, rx_tail, rx_epoch;
static uint32_t consumed_epoch;
static McuUartStream rx_stream;
volatile td_mcu_frame_t g_uart2_tx_frame;
volatile td_mcu_frame_t g_uart2_rx_frame;
volatile uint32_t g_uart2_rx_frame_count;
volatile uint32_t g_uart2_rx_bad_tail_count;
volatile uint32_t g_uart2_rx_overflow_count;
volatile uint32_t g_uart2_rx_timestamp_us;
volatile uint32_t g_uart2_rx_valid;
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
    rx_head = rx_tail = rx_epoch = consumed_epoch = 0U;
    rx_stream = (McuUartStream){0};
    g_uart2_tx_frame = (td_mcu_frame_t){0};
    g_uart2_tx_frame.start_flag = TD_MCU_START;
    g_uart2_tx_frame.end_flag = TD_MCU_END;
    g_uart2_rx_frame = (td_mcu_frame_t){0};
    g_uart2_rx_frame_count = g_uart2_rx_bad_tail_count = 0U;
    g_uart2_rx_overflow_count = g_uart2_rx_timestamp_us = g_uart2_rx_valid = 0U;
    g_uart2_tx_count = g_uart2_rx_count = g_uart2_last_rx = g_uart2_error_count = 0U;
    NVIC_SetPriority(UART9_IRQn, 6U); /* Lower priority than 1ms IMU/I2C. */
    NVIC_ClearPendingIRQ(UART9_IRQn);
    NVIC_EnableIRQ(UART9_IRQn);
    UART9->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_UE;
    previous_send = imu_time_us();
}

/* Main consumes at most 256 bytes per pass so IMU processing gets CPU time.
 * ISR owns head, main owns tail; DMB publishes bytes before advancing head. */
static void receive_poll(void)
{
    for (uint32_t budget = 0U; budget < 256U; ++budget) {
        if (consumed_epoch != rx_epoch) {
            uint32_t saved = __get_PRIMASK();
            __disable_irq();
            rx_tail = rx_head;
            consumed_epoch = rx_epoch;
            __set_PRIMASK(saved);
            rx_stream.used = 0U; /* Never join bytes across a known UART loss. */
        }
        uint32_t tail = rx_tail;
        if (tail == rx_head) break;
        __DMB();
        uint8_t byte = rx_ring[tail];
        __DMB();
        rx_tail = (tail + 1U) & RX_RING_MASK;
        uint32_t epoch = consumed_epoch;
        td_mcu_frame_t frame;
        if (mcu_uart_stream_push(&rx_stream, byte, &frame)) {
            /* Recheck after parsing in case an error IRQ interrupted this pass. */
            uint32_t saved = __get_PRIMASK();
            __disable_irq();
            if (epoch == rx_epoch) {
                g_uart2_rx_frame = frame;
                g_uart2_rx_timestamp_us = imu_time_us();
                ++g_uart2_rx_frame_count;
                g_uart2_rx_valid = 1U;
            }
            __set_PRIMASK(saved);
        }
    }
    g_uart2_rx_bad_tail_count = rx_stream.bad_tail_count;
}

int reserved_uart2_get_rx(td_mcu_frame_t *out)
{
    if (out == 0 || !g_uart2_rx_valid) return 0;
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    *out = g_uart2_rx_frame;
    __set_PRIMASK(saved);
    return 1;
}

void reserved_uart2_set_tx(const td_mcu_frame_t *frame)
{
    if (frame == 0) return;
    g_uart2_tx_frame = *frame;
}

void reserved_uart2_poll(void)
{
    receive_poll();
    uint32_t now = imu_time_us();
    if (!tx_busy && (uint32_t)(now - previous_send) >= RESERVED_UART2_TX_PERIOD_US) {
        previous_send = now; /* Skip missed periods; never send a backlog. */
        ImuData imu;
        imu_get_data(&imu);
        /* Only these three channels are mapped; keep all other values intact.
         * No additional status bits or command semantics are implied. */
        g_uart2_tx_frame.data3 = g_battery_data.filtered_voltage_v;
        g_uart2_tx_frame.data4 = imu.angle_rad.roll;
        g_uart2_tx_frame.data5 = imu.angle_rad.pitch;
        td_mcu_frame_t frame = g_uart2_tx_frame;
        td_mcu_encode(tx_buffer, &frame); /* Static storage remains intact until TC. */
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
        ++rx_epoch;
    }
    if (status & USART_ISR_RXNE_RXFNE) {
        uint8_t byte = (uint8_t)UART9->RDR;
        g_uart2_last_rx = byte;
        ++g_uart2_rx_count;
        if (!(status & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE))) {
            uint32_t head = rx_head;
            uint32_t next = (head + 1U) & RX_RING_MASK;
            if (next == rx_tail) {
                ++g_uart2_rx_overflow_count;
                ++rx_epoch;
            } else {
                rx_ring[head] = byte;
                __DMB();
                rx_head = next;
            }
        }
    }
    if ((status & USART_ISR_TXE_TXFNF) && (UART9->CR1 & USART_CR1_TXEIE_TXFNFIE)) {
        UART9->TDR = tx_buffer[tx_index++];
        if (tx_index >= TD_MCU_FRAME_SIZE) {
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
