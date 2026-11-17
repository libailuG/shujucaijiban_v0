#include "stm32h7xx.h"
#include "imu_bus.h"
#include "motor_bus.h"
#define RING_SIZE 512U
#define RING_MASK (RING_SIZE-1U)
/* Provisional DE setup margin, verify against fitted XL485 and scope on board.
 * Nonblocking: never delay the 1ms IMU processing while waiting. */
#define DE_SETUP_US 2U
typedef struct {
    USART_TypeDef *uart;
    GPIO_TypeDef *gpio;
    uint32_t de, id;
    volatile MotorData *data;
    uint8_t tx[48], ring[RING_SIZE];
    volatile uint32_t head, tail, epoch, state;
    uint32_t consumed_epoch, index, de_since;
    MotorStream stream;
} MotorBus;
volatile MotorData g_motor_left, g_motor_right;
static MotorBus left={.uart=USART3,.gpio=GPIOD,.de=1U<<10,.id=126,.data=&g_motor_left};
static MotorBus right={.uart=USART6,.gpio=GPIOC,.de=1U<<8,.id=127,.data=&g_motor_right};
static uint32_t previous_send;
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
static void uart_init(MotorBus *b,IRQn_Type irq) {
    USART_TypeDef *u=b->uart;
    u->CR1=0; u->CR2=0; u->CR3=0; u->PRESC=0;
    u->BRR=46; /* PCLK 137.5MHz /46 = 2,989,130 baud (-0.362%). */
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
    uart_init(&left,USART3_IRQn); uart_init(&right,USART6_IRQn);
    previous_send=imu_time_us();
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
static void start_stop_frame(MotorBus *b,uint32_t now) {
    if(b->state) { ++b->data->busy_skips; return; }
    MotorFrame f={0}; f.start_flag=0xAAAAAAAAU; f.end_flag=0xEEEEEEEEU;
    f.motor_id=b->id; f.mode=MOTOR_STOP;
    b->data->tx_frame=f; motor_encode(b->tx,&f);
    b->index=0; b->uart->ICR=USART_ICR_TCCF;
    b->gpio->BSRR=b->de; b->de_since=now;
    __DMB(); b->state=1;
}
static void enable_tx(MotorBus *b,uint32_t now) {
    if(b->state==1 && (uint32_t)(now-b->de_since)>=DE_SETUP_US) {
        b->state=2; b->uart->CR1|=USART_CR1_TXEIE_TXFNFIE;
    }
}
void motor_bus_poll(void) {
    receive_poll(&left); receive_poll(&right);
    uint32_t now=imu_time_us();
    if((uint32_t)(now-previous_send)>=MOTOR_PERIOD_US) {
        previous_send=now; /* Nominal 200Hz; skip missed slots, no catch-up burst. */
        start_stop_frame(&left,now); start_stop_frame(&right,now);
    }
    enable_tx(&left,now); enable_tx(&right,now);
}
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
        ++b->epoch; ++b->data->uart_errors;
    }
    if(s&USART_ISR_RXNE_RXFNE) {
        uint8_t byte=(uint8_t)u->RDR; ++b->data->rx_byte_count;
        if(!errors && b->state==0) {
            uint32_t head=b->head,next=(head+1)&RING_MASK;
            if(next==b->tail) { ++b->epoch; ++b->data->overflow_count; }
            else { b->ring[head]=byte; __DMB(); b->head=next; }
        }
    }
    if((s&USART_ISR_TXE_TXFNF)&&(u->CR1&USART_CR1_TXEIE_TXFNFIE)) {
        u->TDR=b->tx[b->index++];
        if(b->index==48) u->CR1=(u->CR1&~USART_CR1_TXEIE_TXFNFIE)|USART_CR1_TCIE;
    }
    if((u->ISR&USART_ISR_TC)&&(u->CR1&USART_CR1_TCIE)) {
        u->CR1&=~USART_CR1_TCIE; u->ICR=USART_ICR_TCCF;
        b->gpio->BSRR=b->de<<16; /* Only TC means last stop bit is complete. */
        ++b->data->tx_count; __DMB(); b->state=0;
    }
    __DSB();
}
void USART3_IRQHandler(void) { uart_irq(&left); }
void USART6_IRQHandler(void) { uart_irq(&right); }
