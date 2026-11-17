#include "stm32h7xx.h"
#include "imu_bus.h"
#include "cascade_left.h"
#include <string.h>
#define RX_SIZE 4096U
#define RX_MASK (RX_SIZE-1U)
#define DE (1UL<<14)
#define DMA_CLEAR (0x3DUL<<22)
#define DMA_ERRORS ((DMA_LISR_TEIF3|DMA_LISR_DMEIF3|DMA_LISR_FEIF3))
/* DMA1 stream3 / DMAMUX1 channel3 request65, isolated AXI SRAM cache lines.
 * Only CPU reads this buffer after startup. Polling NDTR is independent of IDLE.
 * At 2Mbps a full wrap takes >=20.48ms: discard on any poll gap >=20ms. */
static uint8_t rx_dma[RX_SIZE] __attribute__((section(".cascade_dma_rx"), aligned(32)));
static uint8_t scratch[512], tx[CP_MAX_FRAME];
static uint32_t producer, consumer, dma_pos, last_poll, empty_since;
static volatile uint32_t last_activity;
static uint32_t recovering, partial_since, partial_active;
static uint32_t tx_since, request_sequence, wait_sequence, retry_since;
static volatile uint32_t tx_size, tx_index, tx_phase, wait_since;
static uint32_t tx_failed;
static uint8_t wait_type;
static CpParser parser;
static CpPacket packet;
static CascadeReceiver receiver;
static volatile CascadeSnapshot published;
volatile CascadeDiagnostics g_cascade_left_diag;

static void publish(void)
{
    uint32_t saved=__get_PRIMASK(); __disable_irq();
    published=receiver.data;
    __DMB(); __set_PRIMASK(saved);
}
void cascade_left_get_snapshot(CascadeSnapshot *out)
{
    if(!out) return;
    uint32_t saved=__get_PRIMASK(); __disable_irq();
    *out=published; __set_PRIMASK(saved);
    uint32_t elapsed=imu_time_us()-out->received_us;
    out->valid_mask=out->attitude_mask=0;
    if(elapsed>=100000U) out->online=0;
    for(unsigned j=0;j<3;j++) {
        CpRecord *r=&out->records[j]; cp_age_record(r,elapsed);
        if(!out->online) r->flags&=~(CP_VALID|CP_ATTITUDE_VALID);
        if((out->present_mask&(1U<<j)) && (r->flags&CP_VALID) && (!j || r->status==0)) {
            out->valid_mask|=1U<<j;
            if(j && (r->flags&CP_ATTITUDE_VALID)) out->attitude_mask|=1U<<j;
        }
    }
}
static void reset_parser(void) { cp_parser_reset(&parser); partial_active=0; receiver.candidate_count=0; }
static void rx_abort(void)
{
    UART5->CR3&=~USART_CR3_DMAR;
    DMA1_Stream3->CR&=~DMA_SxCR_EN; recovering=1;
    reset_parser(); receiver.data.online=0; publish();
}
static void rx_start(uint32_t now)
{
    DMA1->LIFCR=DMA_CLEAR;
    if(SCB->CCR&SCB_CCR_DC_Msk) SCB_InvalidateDCache_by_Addr((uint32_t *)rx_dma,RX_SIZE);
    DMA1_Stream3->PAR=(uint32_t)&UART5->RDR;
    DMA1_Stream3->M0AR=(uint32_t)rx_dma;
    DMA1_Stream3->NDTR=RX_SIZE;
    DMA1_Stream3->FCR=0;
    DMA1_Stream3->CR=DMA_SxCR_MINC|DMA_SxCR_CIRC|DMA_SxCR_PL_1;
    UART5->RQR=USART_RQR_RXFRQ;
    UART5->ICR=USART_ICR_ORECF|USART_ICR_FECF|USART_ICR_NECF|USART_ICR_PECF;
    producer=consumer=dma_pos=0; last_poll=empty_since=now;
    __DSB(); DMA1_Stream3->CR|=DMA_SxCR_EN; UART5->CR3|=USART_CR3_DMAR;
    recovering=0;
}
static void retry(uint32_t now)
{
    g_cascade_left_diag.state=CASCADE_RETRY; retry_since=now;
}
static void on_packet(uint32_t now,uint32_t age)
{
    if(packet.type==CP_PUSH) {
        if(cascade_receiver_push(&receiver,&packet,now,age)) {
            g_cascade_left_diag.state=CASCADE_STREAMING; publish();
        }
        return;
    }
    if(tx_phase || (g_cascade_left_diag.state!=CASCADE_WAIT_INFO && g_cascade_left_diag.state!=CASCADE_WAIT_START)) return;
    if(packet.type!=(wait_type|CP_REPLY) || packet.sequence!=wait_sequence) return;
    CpInfo info;
    if(packet.length!=sizeof(info)) { ++g_cascade_left_diag.bad_replies; retry(now); return; }
    memcpy(&info,packet.payload,sizeof(info));
    if(info.role!=CP_THIGH || !(info.capabilities&CP_CAP_STREAM) ||
       !cascade_receiver_bind(&receiver,info.uid)) {
        ++g_cascade_left_diag.bad_replies; retry(now); return;
    }
    if(wait_type==CP_INFO) {
        /* Queue START on a later poll, after the response has fully drained. */
        g_cascade_left_diag.state=CASCADE_WAIT_START; wait_type=0; wait_since=now;
    } else {
        g_cascade_left_diag.state=CASCADE_STREAMING; wait_since=now;
    }
}
static void receive_poll(uint32_t now)
{
    if(recovering) { if(!(DMA1_Stream3->CR&DMA_SxCR_EN)) rx_start(now); return; }
    uint32_t errors=UART5->ISR&(USART_ISR_ORE|USART_ISR_FE|USART_ISR_NE|USART_ISR_PE);
    if(errors || (DMA1->LISR&DMA_ERRORS)) {
        if(errors) ++g_cascade_left_diag.uart_errors; else ++g_cascade_left_diag.dma_errors;
        rx_abort(); return;
    }
    if((uint32_t)(now-last_poll)>=20000U) { ++g_cascade_left_diag.overruns; rx_abort(); return; }
    uint32_t pos=(RX_SIZE-DMA1_Stream3->NDTR)&RX_MASK;
    uint32_t delta=(pos-dma_pos)&RX_MASK;
    producer+=delta; dma_pos=pos; last_poll=now;
    if(delta) last_activity=now;
    uint32_t available=producer-consumer;
    if(available>RX_SIZE-sizeof(scratch)) { ++g_cascade_left_diag.overruns; rx_abort(); return; }
    if(!available) { empty_since=now; return; }
    unsigned n=available>sizeof(scratch)?sizeof(scratch):available;
    unsigned offset=consumer&RX_MASK;
    if(n>RX_SIZE-offset) n=RX_SIZE-offset;
    if(SCB->CCR&SCB_CCR_DC_Msk) {
        uint32_t start=(uint32_t)(rx_dma+offset)&~31UL;
        uint32_t end=((uint32_t)(rx_dma+offset+n)+31U)&~31UL;
        SCB_InvalidateDCache_by_Addr((uint32_t *)start,(int32_t)(end-start));
    }
    __DSB(); memcpy(scratch,rx_dma+offset,n); consumer+=n;
    g_cascade_left_diag.rx_bytes+=n;
    for(unsigned i=0;i<n;i++) {
        if(!partial_active) { partial_since=empty_since; partial_active=1; }
        uint32_t old_crc=parser.bad_crc, old_length=parser.bad_length;
        int complete=cp_feed(&parser,scratch[i],&packet);
        if(old_crc!=parser.bad_crc || old_length!=parser.bad_length) receiver.candidate_count=0;
        if(complete) {
            /* BRR69 on 137.5MHz: round actual wire time upward. */
            uint32_t age=(uint32_t)(now-partial_since)+((packet.length+14U)*6900U+1374U)/1375U;
            on_packet(now,age); partial_active=0;
        } else if(!parser.used) partial_active=0;
    }
    if(consumer==producer) empty_since=now;
}
static void request(uint8_t type,uint32_t now)
{
    uint32_t period=2000U;
    wait_sequence=++request_sequence; wait_type=type;
    tx_size=cp_encode(tx,type,wait_sequence,type==CP_STREAM?&period:0,type==CP_STREAM?4:0);
    tx_index=0; tx_since=now; __DMB(); tx_phase=1;
    /* Disable receive while driving RE#/DE high. */
    UART5->CR1&=~USART_CR1_RE;
    UART5->ICR=USART_ICR_TCCF; GPIOB->BSRR=DE;
    ++g_cascade_left_diag.tx_requests;
    g_cascade_left_diag.state=type==CP_INFO?CASCADE_WAIT_INFO:CASCADE_WAIT_START;
}
static void transmit_poll(uint32_t now)
{
    if(tx_failed) {
        if(!tx_phase) { tx_failed=0; rx_abort(); retry(now); }
        return;
    }
    if(!tx_phase) return;
    if((uint32_t)(now-tx_since)>2000U) {
        uint32_t saved=__get_PRIMASK(); __disable_irq();
        if(!tx_phase) { __set_PRIMASK(saved); return; }
        ++g_cascade_left_diag.tx_errors;
        if(tx_index) {
            /* Stop queueing bytes, but retain DE until the in-flight stop bit.
             * Even the timeout path must not release DE on DMA/TXE completion. */
            tx_failed=1; tx_phase=3;
            UART5->CR1=(UART5->CR1&~USART_CR1_TXEIE_TXFNFIE)|USART_CR1_TCIE;
            __set_PRIMASK(saved); return;
        }
        GPIOB->BSRR=DE<<16;
        UART5->CR1=(UART5->CR1&~(USART_CR1_TXEIE_TXFNFIE|USART_CR1_TCIE))|USART_CR1_RE;
        tx_phase=0; __set_PRIMASK(saved); rx_abort(); retry(now); return;
    }
    if(tx_phase==1 && (uint32_t)(now-tx_since)>=2U) {
        tx_phase=2; UART5->CR1|=USART_CR1_TXEIE_TXFNFIE;
    }
}
void UART5_IRQHandler(void)
{
    if(tx_phase==2 && (UART5->ISR&USART_ISR_TXE_TXFNF) && (UART5->CR1&USART_CR1_TXEIE_TXFNFIE)) {
        UART5->TDR=tx[tx_index++]; if(tx_index==tx_size) tx_phase=3;
        if(tx_phase==3) UART5->CR1=(UART5->CR1&~USART_CR1_TXEIE_TXFNFIE)|USART_CR1_TCIE;
    }
    if(tx_phase==3 && (UART5->ISR&USART_ISR_TC) && (UART5->CR1&USART_CR1_TCIE)) {
        UART5->CR1&=~USART_CR1_TCIE; UART5->ICR=USART_ICR_TCCF;
        GPIOB->BSRR=DE<<16; UART5->CR1|=USART_CR1_RE;
        uint32_t now=imu_time_us();
        tx_phase=0; wait_since=now; last_activity=now;
    }
    __DSB();
}
void cascade_left_init(void)
{
    RCC->AHB4ENR|=RCC_AHB4ENR_GPIOBEN;
    RCC->APB1LENR|=RCC_APB1LENR_UART5EN;
    RCC->AHB1ENR|=RCC_AHB1ENR_DMA1EN; (void)RCC->AHB1ENR;
    GPIOB->BSRR=(1UL<<13)|(DE<<16);
    GPIOB->OTYPER&=~((1UL<<12)|(1UL<<13)|DE);
    GPIOB->PUPDR&=~((3UL<<24)|(3UL<<26)|(3UL<<28));
    GPIOB->OSPEEDR|=(3UL<<24)|(3UL<<26)|(3UL<<28);
    GPIOB->AFR[1]=(GPIOB->AFR[1]&~((15UL<<16)|(15UL<<20)))|(14UL<<16)|(14UL<<20);
    GPIOB->MODER=(GPIOB->MODER&~((3UL<<24)|(3UL<<26)|(3UL<<28)))|(2UL<<24)|(2UL<<26)|(1UL<<28);
    RCC->D2CCIP2R&=~RCC_D2CCIP2R_USART28SEL;
    RCC->APB1LRSTR|=RCC_APB1LRSTR_UART5RST; RCC->APB1LRSTR&=~RCC_APB1LRSTR_UART5RST;
    UART5->CR1=0; UART5->CR2=0; UART5->CR3=0; UART5->PRESC=0;
    UART5->BRR=(137500000UL+CP_BAUD/2U)/CP_BAUD;
    DMA1_Stream3->CR=0; DMAMUX1_Channel3->CCR=65U;
    NVIC_SetPriority(UART5_IRQn,6U); NVIC_ClearPendingIRQ(UART5_IRQn); NVIC_EnableIRQ(UART5_IRQn);
    cascade_receiver_init(&receiver); reset_parser();
    UART5->CR1=USART_CR1_TE|USART_CR1_RE|USART_CR1_UE;
    uint32_t now=imu_time_us(); rx_start(now); last_activity=wait_since=now;
    g_cascade_left_diag.state=CASCADE_LISTEN;
}
void cascade_left_poll(void)
{
    uint32_t now=imu_time_us(); receive_poll(now); transmit_poll(now);
    if(partial_active && (uint32_t)(now-partial_since)>20000U) {
        ++g_cascade_left_diag.partial_timeouts; reset_parser();
    }
    uint32_t state=g_cascade_left_diag.state;
    if(!tx_phase && !recovering && consumer==producer) {
        if(state==CASCADE_WAIT_START && wait_type==0 && (uint32_t)(now-last_activity)>=1000U) request(CP_STREAM,now);
        else if((state==CASCADE_WAIT_INFO || state==CASCADE_WAIT_START) && (uint32_t)(now-wait_since)>=80000U) retry(now);
        else if(state==CASCADE_STREAMING && (uint32_t)(now-(receiver.accepted?receiver.data.received_us:wait_since))>=100000U) {
            receiver.data.online=0; publish(); g_cascade_left_diag.state=CASCADE_LISTEN; wait_since=now;
        } else if((state==CASCADE_LISTEN || (state==CASCADE_RETRY && (uint32_t)(now-retry_since)>=100000U)) &&
                  (uint32_t)(now-wait_since)>=50000U && (uint32_t)(now-last_activity)>=50000U) request(CP_INFO,now);
    }
    g_cascade_left_diag.accepted=receiver.accepted; g_cascade_left_diag.rejected=receiver.rejected;
    g_cascade_left_diag.duplicate_push=receiver.duplicate_push; g_cascade_left_diag.skipped_push=receiver.skipped_push;
    g_cascade_left_diag.bound=receiver.bound;
    for(unsigned i=0;i<3;i++) g_cascade_left_diag.new_samples[i]=receiver.new_samples[i];
    g_cascade_left_diag.bad_crc=parser.bad_crc; g_cascade_left_diag.bad_length=parser.bad_length;
}
