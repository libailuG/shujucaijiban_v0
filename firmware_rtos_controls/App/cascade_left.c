#include "stm32h7xx.h"
#include "cascade_left.h"
/* Left body chain on schematic RIGHT IMU3, JP9. UART7 PE8/PE7 AF7, DE PG0. */
#define PORT_UART UART7
#define PORT_GPIO GPIOG
#define DE (1UL<<0)
#define PORT_RX_DMA DMA1_Stream3
#define PORT_DMA_IFCR DMA1->LIFCR
#define PORT_DMA_ISR DMA1->LISR
#define DMA_CLEAR (0x3DUL<<22)
#define DMA_ERRORS (DMA_LISR_TEIF3|DMA_LISR_DMEIF3|DMA_LISR_FEIF3)
#define CASCADE_DMA_SECTION ".cascade_dma_rx"
#define PORT_SIDE_FLAG 0x100U
#define PORT_IRQn UART7_IRQn
#define PORT_IRQHandler UART7_IRQHandler
#define PORT_DIAG g_cascade_left_diag
#define PORT_INIT cascade_left_init
#define PORT_POLL cascade_left_poll
#define PORT_GET_SNAPSHOT cascade_left_get_snapshot
static void port_hardware_init(void)
{
    RCC->AHB4ENR|=RCC_AHB4ENR_GPIOEEN|RCC_AHB4ENR_GPIOGEN;
    RCC->APB1LENR|=RCC_APB1LENR_UART7EN;
    RCC->AHB1ENR|=RCC_AHB1ENR_DMA1EN; (void)RCC->AHB1ENR;
    GPIOG->BSRR=DE<<16;
    GPIOG->OTYPER&=~DE;
    GPIOG->PUPDR&=~3UL;
    GPIOG->OSPEEDR|=3UL;
    GPIOG->MODER=(GPIOG->MODER&~3UL)|1UL;
    GPIOE->BSRR=1UL<<8;
    GPIOE->OTYPER&=~((1UL<<7)|(1UL<<8));
    GPIOE->PUPDR&=~((3UL<<14)|(3UL<<16));
    GPIOE->OSPEEDR|=(3UL<<14)|(3UL<<16);
    GPIOE->AFR[0]=(GPIOE->AFR[0]&~(15UL<<28))|(7UL<<28);
    GPIOE->AFR[1]=(GPIOE->AFR[1]&~15UL)|7UL;
    GPIOE->MODER=(GPIOE->MODER&~((3UL<<14)|(3UL<<16)))|(2UL<<14)|(2UL<<16);
    RCC->D2CCIP2R&=~RCC_D2CCIP2R_USART28SEL;
    RCC->APB1LRSTR|=RCC_APB1LRSTR_UART7RST; RCC->APB1LRSTR&=~RCC_APB1LRSTR_UART7RST;
    DMA1_Stream3->CR=0; DMAMUX1_Channel3->CCR=79U; /* UART7_RX */
}
#include "cascade_port.inc"
