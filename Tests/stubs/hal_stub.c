#include "hal_stub.h"
#include <stdint.h>
#include <string.h>

DMA_Stream_TypeDef g_DMA1_Stream0 = { 0 };
DMA_Stream_TypeDef g_DMA1_Stream1 = { 0 };
DMA_Stream_TypeDef g_DMA1_Stream2 = { 0 };
DMA_Stream_TypeDef g_DMA1_Stream3 = { 0 };
DMA_Stream_TypeDef g_DMA1_Stream4 = { 0 };
DMA_Stream_TypeDef g_DMA1_Stream5 = { 0 };
DMA_Stream_TypeDef g_DMA1_Stream6 = { 0 };
DMA_Stream_TypeDef g_DMA1_Stream7 = { 0 };
GPIO_TypeDef g_GPIOA              = { 0 };
GPIO_TypeDef g_GPIOB              = { 0 };
GPIO_TypeDef g_GPIOC              = { 0 };
GPIO_TypeDef g_GPIOD              = { 0 };
GPIO_TypeDef g_GPIOE              = { 0 };
GPIO_TypeDef g_GPIOF              = { 0 };
GPIO_TypeDef g_GPIOG              = { 0 };
GPIO_TypeDef g_GPIOH              = { 0 };
GPIO_TypeDef g_GPIOI              = { 0 };
GPIO_TypeDef g_GPIOJ              = { 0 };
GPIO_TypeDef g_GPIOK              = { 0 };
TIM_TypeDef g_TIM5                = { 0 };
TIM_TypeDef g_TIM8                = { 0 };
TIM_TypeDef g_TIM12               = { 0 };
TIM_TypeDef g_TIM13               = { 0 };
SPI_TypeDef g_SPI1                = { 0 };
SPI_TypeDef g_SPI2                = { 0 };
SPI_TypeDef g_SPI3                = { 0 };
SPI_TypeDef g_SPI4                = { 0 };
SPI_TypeDef g_SPI5                = { 0 };
USART_TypeDef g_USART1            = { 0 };
USART_TypeDef g_USART2            = { 0 };
USART_TypeDef g_USART3            = { 0 };
USART_TypeDef g_UART4             = { 0 };
USART_TypeDef g_UART5             = { 0 };
USART_TypeDef g_USART6            = { 0 };

uint32_t SystemCoreClock = 480000000U; // Mock system clock

// HAL function implementations with mock pointer checking
void HAL_Delay (uint32_t ms) {
}

uint32_t HAL_GetTick (void) {

    return 0;
}

uint32_t HAL_GetCurrentCPUID (void) {
    return CM7_CPUID;
}

void __disable_irq (void) {
}

void HAL_NVIC_SetPriorityGrouping (uint32_t PriorityGroup) {
}

void HAL_NVIC_SetPriority (IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority) {
}

void HAL_NVIC_EnableIRQ (IRQn_Type IRQn) {
}

void HAL_NVIC_DisableIRQ (IRQn_Type IRQn) {
}

void HAL_NVIC_SystemReset (void) {
}