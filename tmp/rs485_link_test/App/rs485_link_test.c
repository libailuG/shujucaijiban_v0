#include "stm32h7xx.h"
#include "imu_bus.h"
#include "rs485_link_test.h"
#include <string.h>
#define MASK 511U
#define SLOT_US 5000U
#define DE_SETUP_US 2U
typedef struct {
    USART_TypeDef *u; GPIO_TypeDef *gpio;
    uint32_t de, number, sequence, index, de_time;
    volatile uint32_t state, head, tail, epoch;
    uint32_t seen_epoch, used;
    uint8_t tx[48], ring[512], frame[48];
    volatile Rs485LinkStats *stats;
} LinkPort;
volatile Rs485LinkStats g_rs485_jp2, g_rs485_jp3;
volatile uint32_t g_rs485_link_pass;
static LinkPort jp2={.u=USART6,.gpio=GPIOC,.de=1UL<<8,.number=2,.stats=&g_rs485_jp2};
static LinkPort jp3={.u=USART3,.gpio=GPIOD,.de=1UL<<10,.number=3,.stats=&g_rs485_jp3};
static uint32_t last_slot, next_port;
static void put32(uint8_t *p,uint32_t v) {
    for(unsigned i=0;i<4;i++) p[i]=(uint8_t)(v>>(8*i));
}
static uint32_t get32(const uint8_t *p) {
    return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
}
/* Distinct from motor protocol; 00/FF/55/AA and changing bits exercise the wire. */
static void make_frame(uint8_t *p,uint32_t source,uint32_t sequence) {
    memcpy(p,"TEST",4); put32(p+4,source); put32(p+8,sequence);
    for(unsigned i=12;i<44;i++) {
        static const uint8_t pattern[4]={0,255,85,170};
        p[i]=pattern[(i-12)%4]^(uint8_t)(sequence>>(8*((i-12)/8)))^(uint8_t)(i+source);
    }
    memcpy(p+44,"DONE",4);
}
static void pins(GPIO_TypeDef *p,unsigned tx,unsigned rx,unsigned de) {
    uint32_t mask=(1UL<<tx)|(1UL<<rx)|(1UL<<de);
    p->BSRR=(1UL<<tx)|(1UL<<(de+16)); p->OTYPER&=~mask;
    for(unsigned i=0;i<16;i++) if(mask&(1UL<<i)) {
        p->PUPDR&=~(3UL<<(2*i));
        p->OSPEEDR=(p->OSPEEDR&~(3UL<<(2*i)))|(3UL<<(2*i));
        if(i!=de) p->AFR[i/8]=(p->AFR[i/8]&~(15UL<<(4*(i%8))))|(7UL<<(4*(i%8)));
        p->MODER=(p->MODER&~(3UL<<(2*i)))|((i==de?1UL:2UL)<<(2*i));
    }
}
static void uart_init(LinkPort *p,IRQn_Type irq) {
    p->u->CR1=0; p->u->CR2=0; p->u->CR3=0; p->u->PRESC=0;
    /* PCLK1/PCLK2=137.5MHz, oversampling16: BRR46 at 3Mbps. */
    p->u->BRR=(137500000UL+RS485_LINK_TEST_BAUD/2UL)/RS485_LINK_TEST_BAUD;
    NVIC_SetPriority(irq,6); NVIC_ClearPendingIRQ(irq); NVIC_EnableIRQ(irq);
    p->u->CR1=USART_CR1_UE|USART_CR1_TE|USART_CR1_RE|USART_CR1_RXNEIE_RXFNEIE;
}
void rs485_link_test_init(void) {
    RCC->AHB4ENR|=RCC_AHB4ENR_GPIOCEN|RCC_AHB4ENR_GPIODEN;
    RCC->APB1LENR|=RCC_APB1LENR_USART3EN; RCC->APB2ENR|=RCC_APB2ENR_USART6EN;
    (void)RCC->APB2ENR;
    pins(GPIOC,6,7,8); pins(GPIOD,8,9,10);
    RCC->D2CCIP2R&=~(RCC_D2CCIP2R_USART28SEL|RCC_D2CCIP2R_USART16910SEL);
    RCC->APB1LRSTR|=RCC_APB1LRSTR_USART3RST; RCC->APB1LRSTR&=~RCC_APB1LRSTR_USART3RST;
    RCC->APB2RSTR|=RCC_APB2RSTR_USART6RST; RCC->APB2RSTR&=~RCC_APB2RSTR_USART6RST;
    uart_init(&jp2,USART6_IRQn); uart_init(&jp3,USART3_IRQn);
    last_slot=imu_time_us();
}
static void consume(LinkPort *p) {
    for(unsigned budget=0;budget<128;budget++) {
        if(p->seen_epoch!=p->epoch) {
            uint32_t saved=__get_PRIMASK(); __disable_irq();
            p->tail=p->head; p->seen_epoch=p->epoch; __set_PRIMASK(saved); p->used=0;
        }
        uint32_t tail=p->tail; if(tail==p->head) break;
        __DMB(); uint8_t byte=p->ring[tail]; __DMB(); p->tail=(tail+1)&MASK;
        uint32_t epoch=p->seen_epoch;
        p->frame[p->used++]=byte;
        while(p->used>=4 && memcmp(p->frame,"TEST",4)!=0) {
            --p->used; memmove(p->frame,p->frame+1,p->used);
        }
        if(p->used==48) {
            uint8_t expected[48]; uint32_t seq=get32(p->frame+8);
            make_frame(expected,p->number==2?3:2,seq);
            if(memcmp(p->frame,expected,48)!=0) {
                ++p->stats->bad_frame; p->used=47; memmove(p->frame,p->frame+1,47);
            } else {
                uint32_t saved=__get_PRIMASK(); __disable_irq();
                if(epoch==p->epoch) {
                    if(p->stats->valid && seq!=p->stats->last_sequence+1) ++p->stats->sequence_errors;
                    p->stats->last_sequence=seq; p->stats->rx_timestamp_us=imu_time_us();
                    for(unsigned i=0;i<48;i++) p->stats->last_rx[i]=p->frame[i];
                    ++p->stats->rx_count; p->stats->valid=1;
                    g_rs485_link_pass|=p->number==2?1U:2U;
                }
                __set_PRIMASK(saved); p->used=0;
            }
        }
    }
}
static void send(LinkPort *p,uint32_t now) {
    if(p->state) { ++p->stats->busy_skips; return; }
    make_frame(p->tx,p->number,++p->sequence); p->index=0;
    p->u->ICR=USART_ICR_TCCF; p->gpio->BSRR=p->de;
    p->de_time=now; __DMB(); p->state=1;
}
void rs485_link_test_poll(void) {
    consume(&jp2); consume(&jp3);
    uint32_t now=imu_time_us();
    if((uint32_t)(now-last_slot)>=SLOT_US) {
        /* Do not start the other driver if a delayed TX is still in progress. */
        if(!jp2.state && !jp3.state) {
            send(next_port?&jp3:&jp2,now); next_port^=1;
        } else ++(next_port?&g_rs485_jp3:&g_rs485_jp2)->busy_skips;
        last_slot=now;
    }
    LinkPort *ports[2]={&jp2,&jp3};
    for(unsigned i=0;i<2;i++) if(ports[i]->state==1 &&
        (uint32_t)(now-ports[i]->de_time)>=DE_SETUP_US) {
        ports[i]->state=2; ports[i]->u->CR1|=USART_CR1_TXEIE_TXFNFIE;
    }
}
static void irq(LinkPort *p) {
    uint32_t status=p->u->ISR;
    uint32_t errors=status&(USART_ISR_PE|USART_ISR_FE|USART_ISR_NE|USART_ISR_ORE);
    if(errors) {
        p->u->ICR=USART_ICR_PECF|USART_ICR_FECF|USART_ICR_NECF|USART_ICR_ORECF;
        ++p->stats->uart_errors; p->stats->error_flags|=errors; ++p->epoch;
    }
    if(status&USART_ISR_RXNE_RXFNE) {
        uint8_t b=(uint8_t)p->u->RDR; ++p->stats->rx_bytes;
        if(!errors && p->state==0) {
            uint32_t head=p->head,next=(head+1)&MASK;
            if(next==p->tail) { ++p->stats->overflows; ++p->epoch; }
            else { p->ring[head]=b; __DMB(); p->head=next; }
        }
    }
    if((status&USART_ISR_TXE_TXFNF)&&(p->u->CR1&USART_CR1_TXEIE_TXFNFIE)) {
        p->u->TDR=p->tx[p->index++];
        if(p->index==48) p->u->CR1=(p->u->CR1&~USART_CR1_TXEIE_TXFNFIE)|USART_CR1_TCIE;
    }
    if((p->u->ISR&USART_ISR_TC)&&(p->u->CR1&USART_CR1_TCIE)) {
        p->u->CR1&=~USART_CR1_TCIE; p->u->ICR=USART_ICR_TCCF;
        p->gpio->BSRR=p->de<<16; ++p->stats->tx_count; __DMB(); p->state=0;
    }
    __DSB();
}
void USART3_IRQHandler(void) { irq(&jp3); }
void USART6_IRQHandler(void) { irq(&jp2); }
