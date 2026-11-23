#include "stm32h7xx.h"
#include "cascade_right.h"
/* Right body chain on schematic LEFT IMU2, JP5. USART1 PA9/PA10 AF7. */
#define PORT_UART USART1
#define PORT_GPIO GPIOA
#define DE (1UL<<11)
#define PORT_RX_DMA DMA1_Stream4
#define PORT_DMA_IFCR DMA1->HIFCR
#define PORT_DMA_ISR DMA1->HISR
#define DMA_CLEAR 0x3DUL
#define DMA_ERRORS (DMA_HISR_TEIF4|DMA_HISR_DMEIF4|DMA_HISR_FEIF4)
#define CASCADE_DMA_SECTION ".cascade_right_dma_rx"
#define PORT_SIDE_FLAG 0x200U
#define PORT_IRQn USART1_IRQn
#define PORT_IRQHandler USART1_IRQHandler
#define PORT_DIAG g_cascade_right_diag
#define PORT_INIT cascade_right_init
#define PORT_POLL cascade_right_poll
#define PORT_GET_SNAPSHOT cascade_right_get_snapshot
static void port_hardware_init(void)
{
    RCC->AHB4ENR|=RCC_AHB4ENR_GPIOAEN;
    RCC->APB2ENR|=RCC_APB2ENR_USART1EN;
    RCC->AHB1ENR|=RCC_AHB1ENR_DMA1EN; (void)RCC->AHB1ENR;
    GPIOA->BSRR=(1UL<<9)|(DE<<16);
    GPIOA->OTYPER&=~((1UL<<9)|(1UL<<10)|DE);
    GPIOA->PUPDR&=~((3UL<<18)|(3UL<<20)|(3UL<<22));
    GPIOA->OSPEEDR|=(3UL<<18)|(3UL<<20)|(3UL<<22);
    GPIOA->AFR[1]=(GPIOA->AFR[1]&~((15UL<<4)|(15UL<<8)))|(7UL<<4)|(7UL<<8);
    GPIOA->MODER=(GPIOA->MODER&~((3UL<<18)|(3UL<<20)|(3UL<<22)))|(2UL<<18)|(2UL<<20)|(1UL<<22);
    /* Shared with USART6/UART9: retain PCLK2=137.5MHz. */
    RCC->D2CCIP2R&=~RCC_D2CCIP2R_USART16910SEL;
    RCC->APB2RSTR|=RCC_APB2RSTR_USART1RST; RCC->APB2RSTR&=~RCC_APB2RSTR_USART1RST;
    DMA1_Stream4->CR=0; DMAMUX1_Channel4->CCR=41U;
}
#include "cascade_port.inc"
