#include "stm32h7xx.h"
#include "imu_bus.h"
#include "motor_bus.h"
#include "motor_debug_mode.h"
#include "control_mode1.h"
#include "control_mode2.h"
#include "control_mode3.h"
#include "reserved_uart2.h"
#define RING_SIZE 512U
#define RING_MASK (RING_SIZE-1U)
/* Provisional DE setup margin, verify against fitted XL485 and scope on board.
 * Nonblocking: never delay the 1ms IMU processing while waiting. */
#define DE_SETUP_US 2U
/* DMA1 stream0 belongs to UART9. Motors own streams1/2, requests46/72.
 * Full cache lines in AXI SRAM; never place DMA storage in DTCM. */
static uint8_t motor_tx_dma[2][64] __attribute__((section(".motor_dma_tx"), aligned(32)));
#define MOTOR_DMA_CR (DMA_SxCR_DIR_0 | DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE)
typedef struct {
    USART_TypeDef *uart;
    GPIO_TypeDef *gpio;
    uint32_t de, id;
    volatile MotorData *data;
    uint8_t *tx;
    DMA_Stream_TypeDef *dma;
    uint32_t dma_shift;
    uint8_t ring[RING_SIZE];
    volatile uint32_t head, tail, epoch, state;
    uint32_t consumed_epoch, de_since, abort_since;
    uint32_t idle_sample_us, idle_remainder_us;
    MotorStream stream;
    volatile uint32_t completed_us, completed_mode;
} MotorBus;
volatile MotorData g_motor_left, g_motor_right;
/* 置零开关=data1.bit0，mode2只在调试模式的上升沿各发送一次。
 * 零位过程不等mode2 ACK：左TC后等10ms发右；随后持续mode1零扭矩/KP0。
 * 两路各收到静止查询后的新mode1有效反馈，只代表通信恢复，不校验Flash。
 * 开关2保持打开时无论是否完成都禁止扭矩，关闭且两路恢复后才允许控制。
 * 长时间无消息不退出、不重发；请求期间的非零扭矩不缓存。
 */
volatile uint32_t g_motor_zero_state, g_motor_zero_switch;
volatile uint32_t g_motor_zero_left_sent, g_motor_zero_right_sent, g_motor_zero_recovered;
static uint32_t zero_last_switch, zero_started_us, zero_probe_armed;
static uint32_t zero_probe_tx[2], zero_probe_rx[2];
int motor_debug_receive_command(uint32_t mode_value, uint32_t switches, float left_torque, float right_torque)
{
    uint32_t request = mode_value == RESERVED_UART2_DEBUG_MODE_VALUE ? (switches & 1U) : 0U;
    uint32_t rising = request && !zero_last_switch;
    zero_last_switch = request;
    if (mode_value != RESERVED_UART2_DEBUG_MODE_VALUE) {
        g_motor_zero_switch = 0U;
        return motor_debug_set_command(0U, 0.0f, 0.0f);
    }
    if (!motor_debug_set_command(1U, request ? 0.0f : left_torque,
                                     request ? 0.0f : right_torque)) return 0;
    g_motor_zero_switch = request;
    if (rising && (g_motor_zero_state == MOTOR_ZERO_IDLE || g_motor_zero_state == MOTOR_ZERO_DONE)) {
        zero_started_us = imu_time_us();
        zero_probe_armed = g_motor_zero_recovered = 0U;
        g_motor_zero_state = MOTOR_ZERO_QUEUED;
    }
    return 1;
}
uint32_t motor_zero_status_bit(uint32_t now)
{
    if (g_motor_zero_state >= MOTOR_ZERO_QUEUED && g_motor_zero_state <= MOTOR_ZERO_WAIT_FEEDBACK)
        return (((uint32_t)(now - zero_started_us) / 250000U) & 1U) ^ 1U;
    return g_motor_zero_state == MOTOR_ZERO_DONE && g_motor_zero_switch;
}
static uint32_t zero_torque_required(void)
{
    return g_motor_zero_switch || (g_motor_zero_state >= MOTOR_ZERO_QUEUED &&
        g_motor_zero_state <= MOTOR_ZERO_WAIT_FEEDBACK);
}

volatile MotorDiagnostics g_motor_diag_left, g_motor_diag_right;
volatile MotorRxRecent g_motor_left_rx_recent;
static MotorBus left={.uart=USART3,.gpio=GPIOD,.de=1U<<10,.id=126,.data=&g_motor_left,
    .tx=motor_tx_dma[0],.dma=DMA1_Stream1,.dma_shift=6U};
static MotorBus right={.uart=USART6,.gpio=GPIOC,.de=1U<<8,.id=127,.data=&g_motor_right,
    .tx=motor_tx_dma[1],.dma=DMA1_Stream2,.dma_shift=16U};
static uint32_t previous_send, previous_diag;
/* Alternate independent buses every 5ms: each motor retains its 10ms period. */
#define MOTOR_SEND_GAP_US (MOTOR_PERIOD_US / 2U)
static uint32_t next_send_right;
static volatile MotorDiagnostics *diagnostics(MotorBus *b) {
    return b==&left ? &g_motor_diag_left : &g_motor_diag_right;
}
static void sample_rx_pin(MotorBus *b, unsigned pin) {
    volatile MotorDiagnostics *d=diagnostics(b);
    if(b->state==0U) {
        ++d->rx_pin_samples;
        if(!(b->gpio->IDR&(1UL<<pin))) ++d->rx_pin_low_samples;
    }
}
static void pins(GPIO_TypeDef *p,unsigned tx,unsigned rx,unsigned de) {
    uint32_t pins=(1UL<<tx)|(1UL<<rx)|(1UL<<de);
    p->BSRR=(1UL<<tx)|(1UL<<(de+16));
    p->OTYPER &= ~pins;
    for(unsigned i=0;i<16;i++) if(pins&(1UL<<i)) {
        p->PUPDR &= ~(3UL<<(2*i));
        p->OSPEEDR=(p->OSPEEDR&~(3UL<<(2*i)))|(3UL<<(2*i));
        if(i!=de) p->AFR[i/8]=(p->AFR[i/8]&~(15UL<<(4*(i%8))))|(7UL<<(4*(i%8)));
        p->MODER=(p->MODER&~(3UL<<(2*i)))|((i==de?1UL:2UL)<<(2*i));
    }
}
static void dma_init(MotorBus *b,DMAMUX_Channel_TypeDef *mux,uint32_t request,IRQn_Type irq) {
    b->dma->CR=0U;
    DMA1->LIFCR=0x3DU<<b->dma_shift;
    mux->CCR=request;
    b->dma->PAR=(uint32_t)&b->uart->TDR;
    b->dma->M0AR=(uint32_t)b->tx;
    b->dma->FCR=DMA_SxFCR_FEIE;
    b->dma->CR=MOTOR_DMA_CR;
    NVIC_SetPriority(irq,6U);
    NVIC_ClearPendingIRQ(irq); NVIC_EnableIRQ(irq);
}
static void uart_init(MotorBus *b,IRQn_Type irq) {
    USART_TypeDef *u=b->uart;
    u->CR1=0; u->CR2=0; u->CR3=USART_CR3_ONEBIT; u->PRESC=0;
    /* Retain central RX sampling verified during communication repair. */
    /* PCLK 137.5MHz /69 = 1,992,753.62 baud (-0.362% at 2Mbps). */
    u->BRR=(137500000UL+MOTOR_BAUD/2UL)/MOTOR_BAUD;
    NVIC_SetPriority(irq,6U); /* IMU/I2C priority5 can preempt. */
    NVIC_ClearPendingIRQ(irq); NVIC_EnableIRQ(irq);
    u->CR1=USART_CR1_TE|USART_CR1_RE|USART_CR1_RXNEIE_RXFNEIE|USART_CR1_UE;
}
void motor_bus_init(void) {
    RCC->AHB4ENR|=RCC_AHB4ENR_GPIOCEN|RCC_AHB4ENR_GPIODEN;
    RCC->APB1LENR|=RCC_APB1LENR_USART3EN;
    RCC->APB2ENR|=RCC_APB2ENR_USART6EN;
    (void)RCC->APB2ENR;
    pins(GPIOD,8,9,10); pins(GPIOC,6,7,8);
    /* Keep UART9's shared group on PCLK2; no PLL/clock-tree changes. */
    RCC->D2CCIP2R &= ~(RCC_D2CCIP2R_USART28SEL|RCC_D2CCIP2R_USART16910SEL);
    RCC->APB1LRSTR|=RCC_APB1LRSTR_USART3RST; RCC->APB1LRSTR&=~RCC_APB1LRSTR_USART3RST;
    RCC->APB2RSTR|=RCC_APB2RSTR_USART6RST; RCC->APB2RSTR&=~RCC_APB2RSTR_USART6RST;
    left.idle_sample_us=right.idle_sample_us=imu_time_us();
    left.idle_remainder_us=right.idle_remainder_us=0U;
    g_motor_left.rx_idle_ms=g_motor_right.rx_idle_ms=0U;
    RCC->AHB1ENR|=RCC_AHB1ENR_DMA1EN;
    (void)RCC->AHB1ENR;
    dma_init(&left,DMAMUX1_Channel1,46U,DMA1_Stream1_IRQn);
    dma_init(&right,DMAMUX1_Channel2,72U,DMA1_Stream2_IRQn);
    uart_init(&left,USART3_IRQn); uart_init(&right,USART6_IRQn);
    previous_send=imu_time_us();
    next_send_right=0U;
}
static void receive_poll(MotorBus *b) {
    for(unsigned budget=0;budget<128;budget++) {
        if(b->consumed_epoch!=b->epoch) {
            uint32_t saved=__get_PRIMASK(); __disable_irq();
            b->tail=b->head; b->consumed_epoch=b->epoch;
            __set_PRIMASK(saved); b->stream.used=0;
        }
        uint32_t tail=b->tail;
        if(tail==b->head) break;
        __DMB(); uint8_t byte=b->ring[tail]; __DMB();
        b->tail=(tail+1)&RING_MASK;
        uint32_t epoch=b->consumed_epoch;
        MotorFrame f;
        if(motor_stream_push(&b->stream,byte,&f)) {
            uint32_t saved=__get_PRIMASK(); __disable_irq();
            if(epoch==b->epoch) {
                if(f.motor_id==b->id) {
                    b->data->rx_frame=f;
                    b->data->rx_timestamp_us=imu_time_us();
                    b->data->rx_valid=1; ++b->data->rx_frame_count;
                } else ++b->data->wrong_id_count;
            }
            __set_PRIMASK(saved);
        }
    }
    b->data->bad_tail_count=b->stream.bad_tail;
}
static void start_motor_command_frame(MotorBus *b,uint32_t now,uint32_t mode) {
    if(b->state) { ++b->data->busy_skips; return; }
    MotorFrame f={0}; f.start_flag=0xAAAAAAAAU; f.end_flag=0xEEEEEEEEU;
    /* 用户确认的长期静止设定（2026-11-12）：左右电机均保持使能，
     * 使用 mode1 扭矩模式，目标扭矩/速度/位置/KP/KD等控制字段全部为0。
     * 静止、待机、无运动指令时均沿用本设定，禁止使用 mode0，不能回退到mode0。
     * 这是零扭矩使能，不是断电/封波；发送周期10ms，波特率2Mbps。
     * 即使仅需读取反馈，也继续发送本零扭矩帧，不要改成mode0。
     * MotorFrame必须保持零初始化，防止残留控制量进入帧中。
     * 非零命令来自UART9 data2==11调试或控制模式1接口，均受置零抑制；调试KP=50，控制模式1 KP=10。
     * mode2只用于开关2的一次性位置置零，控制字段仍全部为0。 */
    f.motor_id=b->id; f.mode=mode;
    if (g_motor_debug_mode && mode==MOTOR_TORQUE && !zero_torque_required()) {
        f.torque = b==&left ? g_motor_debug_torque_left : g_motor_debug_torque_right;
        f.KP = 50.0f; /* 电机协议中的电流限幅系数，不是位置环增益。 */
    }
    if (!g_motor_debug_mode && mode==MOTOR_TORQUE && !zero_torque_required()) {
        float command[2];
        if (control_mode1_get_command(command) || control_mode2_get_command(command) || control_mode3_get_command(command)) {
            f.torque=command[b==&left ? 0 : 1];
            f.KP=g_uart2_debug_mode==RESERVED_UART2_CONTROL_MODE2_VALUE ? 20.0f : 10.0f;
            if (g_uart2_debug_mode==RESERVED_UART2_CONTROL_MODE3_VALUE && b==&right) {
                f.torque=0.0f; /* Mode3 right torque stays zero; KP matches left (10). */
            }
        }
    }
    b->data->tx_frame=f; motor_encode(b->tx,&f);
    if(SCB->CCR&SCB_CCR_DC_Msk) SCB_CleanDCache_by_Addr((uint32_t *)b->tx,64);
    __DSB();
    b->completed_mode=UINT32_MAX; /* An aborted frame must not reuse an old completion. */
    b->uart->ICR=USART_ICR_TCCF;
    b->gpio->BSRR=b->de; b->de_since=now;
    __DMB(); b->state=1;
}
static void zero_sequence_poll(uint32_t now)
{
    switch (g_motor_zero_state) {
    case MOTOR_ZERO_QUEUED:
        /* Drain existing TX and discard pre-zero parser state, not valid counters. */
        if (!left.state && !right.state) {
            uint32_t saved = __get_PRIMASK(); __disable_irq();
            left.tail=left.head; right.tail=right.head;
            left.stream.used=right.stream.used=0U;
            __set_PRIMASK(saved);
            start_motor_command_frame(&left,now,MOTOR_ZERO);
            g_motor_zero_state=MOTOR_ZERO_LEFT_TX;
        }
        break;
    case MOTOR_ZERO_LEFT_TX:
        if (!left.state && left.completed_mode==MOTOR_ZERO)
            g_motor_zero_state=MOTOR_ZERO_GAP;
        break;
    case MOTOR_ZERO_GAP:
        if ((uint32_t)(now-left.completed_us)>=10000U && !right.state) {
            start_motor_command_frame(&right,now,MOTOR_ZERO);
            g_motor_zero_state=MOTOR_ZERO_RIGHT_TX;
        }
        break;
    case MOTOR_ZERO_RIGHT_TX:
        if (!right.state && right.completed_mode==MOTOR_ZERO) {
            zero_probe_tx[0]=g_motor_left.tx_count; zero_probe_tx[1]=g_motor_right.tx_count;
            zero_probe_armed=0U;
            g_motor_zero_state=MOTOR_ZERO_WAIT_FEEDBACK;
            previous_send=now-MOTOR_SEND_GAP_US; /* Query left now, right 5ms later. */
            next_send_right=0U;
        }
        break;
    case MOTOR_ZERO_WAIT_FEEDBACK: {
        MotorBus *buses[2]={&left,&right};
        for (unsigned i=0;i<2U;++i) {
            uint32_t bit=1UL<<i;
            MotorBus *b=buses[i];
            if (!(zero_probe_armed&bit)) {
                if (b->completed_mode==MOTOR_TORQUE && b->data->tx_count!=zero_probe_tx[i]) {
                    zero_probe_rx[i]=b->data->rx_frame_count;
                    zero_probe_armed|=bit;
                }
            } else if (b->data->rx_frame_count!=zero_probe_rx[i] && b->data->rx_frame.mode==MOTOR_TORQUE) {
                g_motor_zero_recovered|=bit;
            }
        }
        if (g_motor_zero_recovered==3U) g_motor_zero_state=MOTOR_ZERO_DONE;
        break;
    }
    default: break;
    }
}

static void enable_tx(MotorBus *b,uint32_t now) {
    /* Abort without spinning in an IRQ. After DMA stops, allow 100us for
     * TDR/shift register to drain at 2Mbps before returning DE to receive.
     * Failed commands are not counted complete or automatically retried. */
    if(b->state==3 && !(b->dma->CR&DMA_SxCR_EN)) {
        b->abort_since=now; b->state=4;
    }
    if(b->state==4 && (uint32_t)(now-b->abort_since)>=100U) {
        DMA1->LIFCR=0x3DU<<b->dma_shift;
        b->uart->ICR=USART_ICR_TCCF;
        b->gpio->BSRR=b->de<<16;
        __DMB(); b->state=0;
    }
    if(b->state==1 && (uint32_t)(now-b->de_since)>=DE_SETUP_US) {
        DMA1->LIFCR=0x3DU<<b->dma_shift;
        b->dma->NDTR=48U;
        b->state=2; __DMB();
        b->dma->CR|=DMA_SxCR_EN;
        b->uart->CR3|=USART_CR3_DMAT;
    }
}
static void update_rx_idle(MotorBus *b) {
    /* RX ISR also resets these fields: keep timestamp and accumulator atomic.
     * Accumulate short deltas so the 32-bit TIM2 wrap does not reset idle time. */
    uint32_t saved=__get_PRIMASK(); __disable_irq();
    uint32_t now=imu_time_us(), elapsed=now-b->idle_sample_us;
    b->idle_sample_us=now;
    uint32_t remainder=b->idle_remainder_us+elapsed%1000U;
    uint32_t add_ms=elapsed/1000U+remainder/1000U;
    b->idle_remainder_us=remainder%1000U;
    uint32_t idle=b->data->rx_idle_ms;
    b->data->rx_idle_ms=(add_ms>UINT32_MAX-idle)?UINT32_MAX:idle+add_ms;
    __set_PRIMASK(saved);
}
void motor_bus_poll(void) {
    receive_poll(&left); receive_poll(&right);
    update_rx_idle(&left); update_rx_idle(&right);
    uint32_t now=imu_time_us();
    if((uint32_t)(now-previous_diag)>=11U) {
        previous_diag=now; sample_rx_pin(&left,9); sample_rx_pin(&right,7);
    }
    zero_sequence_poll(now);
    uint32_t normal_allowed = g_motor_zero_state==MOTOR_ZERO_IDLE || g_motor_zero_state>=MOTOR_ZERO_WAIT_FEEDBACK;
    if(normal_allowed && (uint32_t)(now-previous_send)>=MOTOR_SEND_GAP_US) {
        previous_send=now; /* Skip missed slots, no catch-up burst. */
        start_motor_command_frame(next_send_right ? &right : &left,now,MOTOR_TORQUE);
        next_send_right^=1U;
    }
    enable_tx(&left,now); enable_tx(&right,now);
}
/* 电机安装朝向（用户确认）：左侧电机正转时下落，右侧电机正转时上升。
 * 左侧=JP3/USART3/ID126，右侧=JP2/USART6/ID127。
 * 本接口保留电机原始反馈符号，不在读取时做方向取反。
 */
int motor_get_feedback(unsigned is_right,MotorFrame *out) {
    volatile MotorData *d=is_right?&g_motor_right:&g_motor_left;
    if(out==0 || !d->rx_valid) return 0;
    uint32_t saved=__get_PRIMASK(); __disable_irq();
    *out=d->rx_frame; __set_PRIMASK(saved); return 1;
}
static void uart_irq(MotorBus *b) {
    USART_TypeDef *u=b->uart; uint32_t s=u->ISR;
    uint32_t errors=s&(USART_ISR_PE|USART_ISR_FE|USART_ISR_NE|USART_ISR_ORE);
    if(errors) {
        u->ICR=USART_ICR_PECF|USART_ICR_FECF|USART_ICR_NECF|USART_ICR_ORECF;
        ++b->epoch; ++b->data->uart_errors; diagnostics(b)->error_flags_or|=errors;
    }
    if(s&USART_ISR_RXNE_RXFNE) {
        uint8_t byte=(uint8_t)u->RDR; ++b->data->rx_byte_count;
        if(b==&left) {
            uint32_t index=g_motor_left_rx_recent.write_index;
            g_motor_left_rx_recent.bytes[index]=byte;
            __DMB();
            g_motor_left_rx_recent.write_index=(index+1U)&(MOTOR_RX_RECENT_SIZE-1U);
            if(g_motor_left_rx_recent.valid_count<MOTOR_RX_RECENT_SIZE)
                ++g_motor_left_rx_recent.valid_count;
            ++g_motor_left_rx_recent.total_count;
        }
        volatile MotorDiagnostics *d=diagnostics(b);
        d->last_rx_byte=byte; if(b->state) ++d->rx_during_tx;
        /* "Any data" includes bad frames, UART-error bytes and wrong IDs. */
        b->idle_sample_us=imu_time_us(); b->idle_remainder_us=0U;
        b->data->rx_idle_ms=0U;
        if(!errors && b->state==0) {
            uint32_t head=b->head,next=(head+1)&RING_MASK;
            if(next==b->tail) { ++b->epoch; ++b->data->overflow_count; }
            else { b->ring[head]=byte; __DMB(); b->head=next; }
        }
    }
    if((u->ISR&USART_ISR_TC)&&(u->CR1&USART_CR1_TCIE)) {
        u->CR1&=~USART_CR1_TCIE; u->ICR=USART_ICR_TCCF;
        b->gpio->BSRR=b->de<<16; /* Only TC means last stop bit is complete. */
        volatile MotorDiagnostics *d=diagnostics(b);
        b->completed_us=imu_time_us(); b->completed_mode=b->tx[8];
        if (b->completed_mode==MOTOR_ZERO) {
            if (b==&left) ++g_motor_zero_left_sent; else ++g_motor_zero_right_sent;
        }
        uint32_t elapsed=b->completed_us-b->de_since;
        d->last_tx_us=elapsed;
        if(elapsed>d->max_tx_us) d->max_tx_us=elapsed;
        ++b->data->tx_count; __DMB(); b->state=0;
    }
    __DSB();
}
void USART3_IRQHandler(void) { uart_irq(&left); }
void USART6_IRQHandler(void) { uart_irq(&right); }

static void motor_dma_irq(MotorBus *b) {
    uint32_t status=(DMA1->LISR>>b->dma_shift)&0x3DU;
    DMA1->LIFCR=0x3DU<<b->dma_shift;
    if(status&0x0DU) { /* FIFO, direct-mode or transfer error. */
        b->uart->CR3&=~USART_CR3_DMAT;
        b->uart->CR1&=~USART_CR1_TCIE;
        b->dma->CR&=~DMA_SxCR_EN;
        ++b->data->tx_dma_errors;
        b->state=3;
    } else if(status&0x20U) {
        b->uart->CR3&=~USART_CR3_DMAT;
        /* DMA TC only means TDR loaded. UART TC owns DE and completion. */
        b->uart->CR1|=USART_CR1_TCIE;
    }
    __DSB();
}
void DMA1_Stream1_IRQHandler(void) { motor_dma_irq(&left); }
void DMA1_Stream2_IRQHandler(void) { motor_dma_irq(&right); }

