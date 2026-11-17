#include "stm32h7xx.h"
#include "cascade_left.h"
/* Left body chain on schematic RIGHT IMU2, JP6. UART5 PB13/PB12 AF14. */
#define PORT_UART UART5
#define PORT_GPIO GPIOB
#define DE (1UL<<14)
#define PORT_RX_DMA DMA1_Stream3
#define PORT_DMA_IFCR DMA1->LIFCR
#define PORT_DMA_ISR DMA1->LISR
#define DMA_CLEAR (0x3DUL<<22)
#define DMA_ERRORS (DMA_LISR_TEIF3|DMA_LISR_DMEIF3|DMA_LISR_FEIF3)
#define CASCADE_DMA_SECTION ".cascade_dma_rx"
#define PORT_SIDE_FLAG 0x100U
#define PORT_IRQn UART5_IRQn
#define PORT_IRQHandler UART5_IRQHandler
#define PORT_DIAG g_cascade_left_diag
#define PORT_INIT cascade_left_init
#define PORT_POLL cascade_left_poll
#define PORT_GET_SNAPSHOT cascade_left_get_snapshot
static void port_hardware_init(void)
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
    DMA1_Stream3->CR=0; DMAMUX1_Channel3->CCR=65U;
}
#include "cascade_port.inc"
