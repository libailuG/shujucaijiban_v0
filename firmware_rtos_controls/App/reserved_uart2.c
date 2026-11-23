#include "stm32h7xx.h"
#include "imu_bus.h"
#include "reserved_uart2.h"
#include "mcu_uart_stream.h"
#include "normal_mode.h"
#include "motor_debug_mode.h"
#include "imu_debug_mode.h"
#include "control_mode1.h"
#include "control_mode2.h"
#include "control_mode3.h"
#include "motor_bus.h"
#include "acquisition_mode.h"

#define RX_RING_SIZE 1024U
#define RX_RING_MASK (RX_RING_SIZE - 1U)
/* board.sct places this buffer in DMA-accessible AXI SRAM, outside DTCM.
 * Reserve complete cache lines so cache maintenance cannot touch other data.
 * DMA1 stream0 / DMAMUX1 channel0 are owned by UART9 TX (request 117). */
#define TX_DMA_CLEAR (DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0)
static uint8_t tx_buffer[64] __attribute__((section(".uart9_dma_tx"), aligned(32)));
static volatile uint32_t tx_dma_error;
static uint8_t rx_ring[RX_RING_SIZE];
static volatile uint32_t rx_head, rx_tail, rx_epoch;
static uint32_t consumed_epoch;
static McuUartStream rx_stream;
volatile uint32_t g_uart2_debug_mode;
volatile td_mcu_frame_t g_uart2_tx_frame;
volatile td_mcu_frame_t g_uart2_rx_frame;
volatile uint32_t g_uart2_rx_frame_count;
volatile uint32_t g_uart2_rx_bad_tail_count;
volatile uint32_t g_uart2_rx_overflow_count;
volatile uint32_t g_uart2_rx_timestamp_us;
volatile uint32_t g_uart2_rx_valid;
static volatile uint32_t tx_busy;
static uint32_t previous_send;
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
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    (void)RCC->AHB1ENR;
    DMA1_Stream0->CR = 0U;
    DMA1->LIFCR = TX_DMA_CLEAR;
    DMAMUX1_Channel0->CCR = 117U; /* STM32H723 UART9_TX. */
    DMA1_Stream0->PAR = (uint32_t)&UART9->TDR;
    DMA1_Stream0->M0AR = (uint32_t)tx_buffer;
    DMA1_Stream0->FCR = DMA_SxFCR_FEIE; /* Direct mode, byte transfers. */
    DMA1_Stream0->CR = DMA_SxCR_DIR_0 | DMA_SxCR_MINC |
        DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
    NVIC_SetPriority(DMA1_Stream0_IRQn, 6U);
    NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn);
    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    tx_busy = tx_dma_error = 0U;
    rx_head = rx_tail = rx_epoch = consumed_epoch = 0U;
    rx_stream = (McuUartStream){0};
    g_uart2_debug_mode = 0U;
    acquisition_mode_set_enabled(0U);
#if !defined(ZHUBAN_RTOS_PHASE1) || defined(ZHUBAN_RTOS_CONTROLS)
    control_mode1_set_torque_enabled(0U);
    control_mode2_set_torque_enabled(0U);
    control_mode3_set_torque_enabled(0U);
    control_mode2_reset(); control_mode3_reset();
    control_mode1_set_observation_enabled(0U);
#endif
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
            uint32_t accepted = 0U;
            uint32_t saved = __get_PRIMASK();
            __disable_irq();
            if (epoch == rx_epoch) {
                g_uart2_rx_frame = frame;
                g_uart2_rx_timestamp_us = imu_time_us();
                ++g_uart2_rx_frame_count;
                g_uart2_rx_valid = 1U;
                accepted = 1U;
            }
            __set_PRIMASK(saved);
            if (accepted) {
                uint32_t requested_mode = (frame.data1 & RESERVED_UART2_STATIONARY_MASK) ? 0U : reserved_uart2_accept_mode(frame.data2);
#if defined(ZHUBAN_RTOS_PHASE1) && !defined(ZHUBAN_RTOS_CONTROLS)
                /* Phase 1 accepts sensor and acquisition modes only. */
                g_uart2_debug_mode = requested_mode >= RESERVED_UART2_IMU_MODE_VALUE ? requested_mode : 0U;
                acquisition_mode_set_enabled(g_uart2_debug_mode==RESERVED_UART2_ACQUISITION_VALUE && (frame.data1 & 1U));
#else
#ifdef ZHUBAN_RTOS_CONTROLS
                motor_bus_rtos_clear_command();
#endif
                if (motor_debug_receive_command(requested_mode, frame.data1, frame.data8, frame.data9)) {
                    if (g_uart2_debug_mode != requested_mode) { control_mode1_reset(); control_mode2_reset(); control_mode3_reset(); }
                    /* Invalid settings never prevent stop/mode acceptance. */
                    if (requested_mode==RESERVED_UART2_CONTROL_MODE3_VALUE && (frame.data1 & (1U<<2)))
                        (void)control_mode3_set_parameters(frame.data8, frame.data9);
#ifdef ZHUBAN_RTOS_CONTROLS
                    uint32_t mode_irq = __get_PRIMASK();
                    __disable_irq();
                    motor_bus_mode3_track_replies(requested_mode==RESERVED_UART2_CONTROL_MODE3_VALUE);
#endif
                    g_uart2_debug_mode = requested_mode;
#ifdef ZHUBAN_RTOS_CONTROLS
                    __set_PRIMASK(mode_irq);
#endif
                    if (requested_mode==RESERVED_UART2_CONTROL_MODE2_VALUE)
                        control_mode2_set_output_scale(frame.data3);
                    control_mode3_set_torque_enabled(requested_mode==RESERVED_UART2_CONTROL_MODE3_VALUE && (frame.data1 & 1U));
                    control_mode2_set_torque_enabled(requested_mode==RESERVED_UART2_CONTROL_MODE2_VALUE && (frame.data1 & 1U));
                    control_mode1_set_observation_enabled(
                        requested_mode==RESERVED_UART2_CONTROL_MODE1_VALUE &&
                        (frame.data1 & RESERVED_UART2_CONTROL1_OBSERVATION_MASK));
                    acquisition_mode_set_enabled(requested_mode==RESERVED_UART2_ACQUISITION_VALUE && (frame.data1 & 1U));
                    control_mode1_set_torque_enabled(
                        g_uart2_debug_mode==RESERVED_UART2_CONTROL_MODE1_VALUE &&
                        (frame.data1 & RESERVED_UART2_CONTROL1_TORQUE_MASK));
                }
#endif
            }
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
    /* Error abort is nonblocking; reuse storage only after DMA has stopped. */
    if (tx_dma_error && !(DMA1_Stream0->CR & DMA_SxCR_EN)) {
        DMA1->LIFCR = TX_DMA_CLEAR;
        tx_dma_error = 0U;
        tx_busy = 0U;
    }
    receive_poll();
#if !defined(ZHUBAN_RTOS_PHASE1) || defined(ZHUBAN_RTOS_CONTROLS)
    control_mode1_poll(); /* 非控制模式时清空策略及保持命令。 */
    control_mode2_poll();
    control_mode3_poll();
#endif
    acquisition_mode_poll();
    uint32_t now = imu_time_us();
    if (!tx_busy && (uint32_t)(now - previous_send) >= RESERVED_UART2_TX_PERIOD_US) {
        previous_send = now; /* Skip missed periods; never send a backlog. */
#if defined(ZHUBAN_RTOS_PHASE1) && !defined(ZHUBAN_RTOS_CONTROLS)
        uint32_t bit0 = g_uart2_debug_mode == RESERVED_UART2_ACQUISITION_VALUE && acquisition_mode_is_enabled();
        g_uart2_tx_frame.data1 = (g_uart2_tx_frame.data1 &
            ~(RESERVED_UART2_ZERO_SWITCH_MASK | RESERVED_UART2_CONTROL1_OBSERVATION_MASK | RESERVED_UART2_SENSOR_STATUS_MASK)) |
            (bit0 ? RESERVED_UART2_ZERO_SWITCH_MASK : 0U);
#else
        /* bit0 echoes torque permission in control modes, zero status in mode 11. */
        uint32_t bit0 = g_uart2_debug_mode == RESERVED_UART2_CONTROL_MODE1_VALUE ?
            g_control_mode1_torque_enabled :
            (g_uart2_debug_mode == RESERVED_UART2_CONTROL_MODE2_VALUE ? g_control_mode2_torque_enabled :
            (g_uart2_debug_mode == RESERVED_UART2_CONTROL_MODE3_VALUE ? g_control_mode3_torque_enabled :
            (g_uart2_debug_mode == RESERVED_UART2_ACQUISITION_VALUE ? acquisition_mode_is_enabled() :
            (g_uart2_debug_mode == RESERVED_UART2_DEBUG_MODE_VALUE ? motor_zero_status_bit(now) : 0U))));
        g_uart2_tx_frame.data1 = (g_uart2_tx_frame.data1 &
            ~(RESERVED_UART2_ZERO_SWITCH_MASK | RESERVED_UART2_CONTROL1_OBSERVATION_MASK | RESERVED_UART2_SENSOR_STATUS_MASK)) |
            (bit0 ? RESERVED_UART2_ZERO_SWITCH_MASK : 0U) |
            ((g_uart2_debug_mode==RESERVED_UART2_CONTROL_MODE1_VALUE && g_control_mode1_observation_enabled) ?
             RESERVED_UART2_CONTROL1_OBSERVATION_MASK : 0U);
#endif
        g_uart2_tx_frame.data2 = g_uart2_debug_mode;
        if (g_uart2_debug_mode == RESERVED_UART2_ACQUISITION_VALUE) {
            acquisition_mode_fill_feedback(&g_uart2_tx_frame);
        } else if (g_uart2_debug_mode == RESERVED_UART2_IMU_MODE_VALUE) {
            imu_debug_mode_fill_feedback(&g_uart2_tx_frame);
        } else if (g_uart2_debug_mode >= RESERVED_UART2_LEFT_THIGH_VALUE &&
                   g_uart2_debug_mode <= RESERVED_UART2_RIGHT_PRESSURE_VALUE) {
            cascade_debug_mode_fill_feedback(&g_uart2_tx_frame, g_uart2_debug_mode);
#if !defined(ZHUBAN_RTOS_PHASE1) || defined(ZHUBAN_RTOS_CONTROLS)
        } else if (g_uart2_debug_mode == RESERVED_UART2_CONTROL_MODE1_VALUE) {
            control_mode1_fill_feedback(&g_uart2_tx_frame);
        } else if (g_uart2_debug_mode == RESERVED_UART2_CONTROL_MODE2_VALUE) {
            control_mode2_fill_feedback(&g_uart2_tx_frame);
        } else if (g_uart2_debug_mode == RESERVED_UART2_CONTROL_MODE3_VALUE) {
            control_mode3_fill_feedback(&g_uart2_tx_frame);
        } else if (g_motor_debug_mode) {
            motor_debug_mode_fill_feedback(&g_uart2_tx_frame);
#endif
        } else {
            /* 普通模式反馈。 */
            normal_mode_fill_feedback(&g_uart2_tx_frame);
        }
        td_mcu_frame_t frame = g_uart2_tx_frame;
        td_mcu_encode(tx_buffer, &frame); /* Static storage remains intact until TC. */
        if (SCB->CCR & SCB_CCR_DC_Msk)
            SCB_CleanDCache_by_Addr((uint32_t *)tx_buffer, sizeof(tx_buffer));
        __DSB();
        DMA1->LIFCR = TX_DMA_CLEAR;
        DMA1_Stream0->NDTR = TD_MCU_FRAME_SIZE;
        tx_busy = 1U;
        __DMB();
        UART9->ICR = USART_ICR_TCCF;
        DMA1_Stream0->CR |= DMA_SxCR_EN;
        UART9->CR3 |= USART_CR3_DMAT;
    }
}
/* DMA completion precedes the final stop bit. Keep tx_busy until UART TC. */
void DMA1_Stream0_IRQHandler(void)
{
    uint32_t status = DMA1->LISR;
    DMA1->LIFCR = TX_DMA_CLEAR;
    if (status & (DMA_LISR_TEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_FEIF0)) {
        UART9->CR3 &= ~USART_CR3_DMAT;
        UART9->CR1 &= ~USART_CR1_TCIE;
        DMA1_Stream0->CR &= ~DMA_SxCR_EN;
        ++g_uart2_error_count;
        tx_dma_error = 1U;
    } else if (status & DMA_LISR_TCIF0) {
        UART9->CR3 &= ~USART_CR3_DMAT;
        UART9->CR1 |= USART_CR1_TCIE;
    }
    __DSB();
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
    if ((UART9->ISR & USART_ISR_TC) && (UART9->CR1 & USART_CR1_TCIE)) {
        UART9->CR1 &= ~USART_CR1_TCIE;
        UART9->ICR = USART_ICR_TCCF;
        ++g_uart2_tx_count;
        __DMB();
        tx_busy = 0U;
    }
    __DSB();
}


