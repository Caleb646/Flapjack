#include "hal.h"
#include <stdint.h>
#include <string.h>

// clang-format off

void (*HAL_Delay_Mock)(uint32_t ms) = NULL;
uint32_t (*HAL_GetTick_Mock)(void) = NULL;
uint32_t (*HAL_GetCurrentCPUID_Mock)(void) = NULL;
void (*__disable_irq_Mock)(void) = NULL;

// Timer function pointers
HAL_StatusTypeDef (*HAL_TIM_PWM_Init_Mock)(TIM_HandleTypeDef* htim) = NULL;
HAL_StatusTypeDef (*HAL_TIM_PWM_ConfigChannel_Mock)(TIM_HandleTypeDef* htim, TIM_OC_InitTypeDef* sConfig, uint32_t Channel) = NULL;
HAL_StatusTypeDef (*HAL_TIM_PWM_Start_Mock)(TIM_HandleTypeDef* htim, uint32_t Channel) = NULL;
HAL_StatusTypeDef (*HAL_TIM_PWM_Stop_Mock)(TIM_HandleTypeDef* htim, uint32_t Channel) = NULL;
HAL_StatusTypeDef (*HAL_TIM_PWM_Start_DMA_Mock)(TIM_HandleTypeDef* htim, uint32_t Channel, uint32_t* pData, uint16_t Length) = NULL;
HAL_StatusTypeDef (*HAL_TIM_PWM_Stop_DMA_Mock)(TIM_HandleTypeDef* htim, uint32_t Channel) = NULL;
void (*HAL_TIM_ErrorCallback_Mock)(TIM_HandleTypeDef* htim) = NULL;
void (*HAL_TIM_PWM_PulseFinishedCallback_Mock)(TIM_HandleTypeDef* htim) = NULL;
void (*HAL_TIM_PWM_PulseFinishedHalfCpltCallback_Mock)(TIM_HandleTypeDef* htim) = NULL;

// SPI function pointers
HAL_StatusTypeDef (*HAL_SPI_Init_Mock)(SPI_HandleTypeDef* hspi) = NULL;
HAL_StatusTypeDef (*HAL_SPI_DeInit_Mock)(SPI_HandleTypeDef* hspi) = NULL;
HAL_StatusTypeDef (*HAL_SPI_Transmit_Mock)(SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t Size, uint32_t Timeout) = NULL;
HAL_StatusTypeDef (*HAL_SPI_Receive_Mock)(SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t Size, uint32_t Timeout) = NULL;
HAL_StatusTypeDef (*HAL_SPI_TransmitReceive_Mock)(SPI_HandleTypeDef* hspi, const uint8_t* pTxData, uint8_t* pRxData, uint16_t Size, uint32_t Timeout) = NULL;

// UART function pointers
HAL_StatusTypeDef (*HAL_UART_Init_Mock)(UART_HandleTypeDef* huart) = NULL;
HAL_StatusTypeDef (*HAL_UART_DeInit_Mock)(UART_HandleTypeDef* huart) = NULL;
HAL_StatusTypeDef (*HAL_UART_Transmit_Mock)(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size, uint32_t Timeout) = NULL;
HAL_StatusTypeDef (*HAL_UART_Receive_Mock)(UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size, uint32_t Timeout) = NULL;
HAL_StatusTypeDef (*HAL_UART_Receive_IT_Mock)(UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size) = NULL;
void (*HAL_UART_IRQHandler_Mock)(UART_HandleTypeDef* huart) = NULL;
HAL_StatusTypeDef (*HAL_UARTEx_SetTxFifoThreshold_Mock)(UART_HandleTypeDef* huart, uint32_t Threshold) = NULL;
HAL_StatusTypeDef (*HAL_UARTEx_SetRxFifoThreshold_Mock)(UART_HandleTypeDef* huart, uint32_t Threshold) = NULL;
HAL_StatusTypeDef (*HAL_UARTEx_DisableFifoMode_Mock)(UART_HandleTypeDef* huart) = NULL;
void (*HAL_UART_TxCpltCallback_Mock)(UART_HandleTypeDef* huart) = NULL;
void (*HAL_UART_RxCpltCallback_Mock)(UART_HandleTypeDef* huart) = NULL;
void (*HAL_UART_ErrorCallback_Mock)(UART_HandleTypeDef* huart) = NULL;

// DMA function pointers
HAL_StatusTypeDef (*HAL_DMA_Init_Mock)(DMA_HandleTypeDef* hdma) = NULL;
HAL_StatusTypeDef (*HAL_DMA_DeInit_Mock)(DMA_HandleTypeDef* hdma) = NULL;
void (*HAL_DMA_IRQHandler_Mock)(DMA_HandleTypeDef* hdma) = NULL;
HAL_StatusTypeDef (*HAL_DMA_Start_Mock)(DMA_HandleTypeDef* hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength) = NULL;
HAL_StatusTypeDef (*HAL_DMA_Start_IT_Mock)(DMA_HandleTypeDef* hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength) = NULL;
HAL_StatusTypeDef (*HAL_DMA_Abort_Mock)(DMA_HandleTypeDef* hdma) = NULL;
uint32_t (*HAL_DMA_GetError_Mock)(const DMA_HandleTypeDef* hdma) = NULL;

// RCC function pointers
HAL_StatusTypeDef (*HAL_RCCEx_PeriphCLKConfig_Mock)(RCC_PeriphCLKInitTypeDef* PeriphClkInit) = NULL;

// GPIO function pointers
void (*HAL_GPIO_WritePin_Mock)(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t PinState) = NULL;
void (*HAL_GPIO_Init_Mock)(GPIO_TypeDef* GPIOx, const GPIO_InitTypeDef* GPIO_Init) = NULL;
void (*HAL_GPIO_DeInit_Mock)(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin) = NULL;
void (*HAL_GPIO_TogglePin_Mock)(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) = NULL;

// NVIC function pointers
void (*HAL_NVIC_SetPriority_Mock)(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority) = NULL;
void (*HAL_NVIC_EnableIRQ_Mock)(IRQn_Type IRQn) = NULL;
void (*HAL_NVIC_DisableIRQ_Mock)(IRQn_Type IRQn) = NULL;

// clang-format on

extern DMA_Stream_TypeDef g_DMA1_Stream0 = { 0 };
extern DMA_Stream_TypeDef g_DMA1_Stream1 = { 0 };
extern DMA_Stream_TypeDef g_DMA1_Stream2 = { 0 };
extern DMA_Stream_TypeDef g_DMA1_Stream3 = { 0 };
extern DMA_Stream_TypeDef g_DMA1_Stream4 = { 0 };
extern DMA_Stream_TypeDef g_DMA1_Stream5 = { 0 };
extern DMA_Stream_TypeDef g_DMA1_Stream6 = { 0 };
extern DMA_Stream_TypeDef g_DMA1_Stream7 = { 0 };

extern GPIO_TypeDef g_GPIOA = { 0 };
extern GPIO_TypeDef g_GPIOB = { 0 };
extern GPIO_TypeDef g_GPIOC = { 0 };
extern GPIO_TypeDef g_GPIOD = { 0 };
extern GPIO_TypeDef g_GPIOE = { 0 };
extern GPIO_TypeDef g_GPIOF = { 0 };
extern GPIO_TypeDef g_GPIOG = { 0 };
extern GPIO_TypeDef g_GPIOH = { 0 };
extern GPIO_TypeDef g_GPIOI = { 0 };
extern GPIO_TypeDef g_GPIOJ = { 0 };
extern GPIO_TypeDef g_GPIOK = { 0 };

extern TIM_TypeDef g_TIM1  = { 0 };
extern TIM_TypeDef g_TIM2  = { 0 };
extern TIM_TypeDef g_TIM3  = { 0 };
extern TIM_TypeDef g_TIM4  = { 0 };
extern TIM_TypeDef g_TIM5  = { 0 };
extern TIM_TypeDef g_TIM6  = { 0 };
extern TIM_TypeDef g_TIM7  = { 0 };
extern TIM_TypeDef g_TIM8  = { 0 };
extern TIM_TypeDef g_TIM12 = { 0 };
extern TIM_TypeDef g_TIM13 = { 0 };
extern TIM_TypeDef g_TIM14 = { 0 };
extern TIM_TypeDef g_TIM15 = { 0 };
extern TIM_TypeDef g_TIM16 = { 0 };
extern TIM_TypeDef g_TIM17 = { 0 };

extern SPI_TypeDef g_SPI1 = { 0 };
extern SPI_TypeDef g_SPI2 = { 0 };
extern SPI_TypeDef g_SPI3 = { 0 };
extern SPI_TypeDef g_SPI4 = { 0 };
extern SPI_TypeDef g_SPI5 = { 0 };

extern USART_TypeDef g_USART1 = { 0 };
extern USART_TypeDef g_USART2 = { 0 };
extern USART_TypeDef g_USART3 = { 0 };
extern USART_TypeDef g_UART4  = { 0 };
extern USART_TypeDef g_UART5  = { 0 };
extern USART_TypeDef g_USART6 = { 0 };

uint32_t SystemCoreClock = 480000000U; // Mock system clock

// HAL function implementations with mock pointer checking
void HAL_Delay (uint32_t ms) {

    if (HAL_Delay_Mock != NULL) {
        HAL_Delay_Mock (ms);
        return;
    }

    // Default weak implementation - do nothing
}

uint32_t HAL_GetTick (void) {

    if (HAL_GetTick_Mock != NULL) {
        return HAL_GetTick_Mock ();
    }

    return 0; // Default weak implementation
}

uint32_t HAL_GetCurrentCPUID (void) {

    if (HAL_GetCurrentCPUID_Mock != NULL) {
        return HAL_GetCurrentCPUID_Mock ();
    }

    return CM7_CPUID; // Default weak implementation
}

void __disable_irq (void) {

    if (__disable_irq_Mock != NULL) {
        __disable_irq_Mock ();
        return;
    }
}

void __enable_irq (void) {
}

HAL_StatusTypeDef HAL_HSEM_FastTake (uint32_t SemID) {
    (void)SemID;
    return HAL_OK;
}

void HAL_HSEM_Release (uint32_t SemID, uint32_t ProcessID) {
    (void)SemID;
    (void)ProcessID;
}

void HAL_HSEM_ActivateNotification (uint32_t SemMask) {
    (void)SemMask;
}

void HAL_HSEM_DeactivateNotification (uint32_t SemMask) {
    (void)SemMask;
}

void HAL_Init (void) {
}

void HAL_IncTick (void) {
}

// DMA HAL functions
void HAL_DMA_IRQHandler (DMA_HandleTypeDef* hdma) {

    if (HAL_DMA_IRQHandler_Mock != NULL) {
        HAL_DMA_IRQHandler_Mock (hdma);
        return;
    }

    // Default weak implementation - do nothing
}

HAL_StatusTypeDef HAL_DMA_Init (DMA_HandleTypeDef* hdma) {

    if (HAL_DMA_Init_Mock != NULL) {
        return HAL_DMA_Init_Mock (hdma);
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_DMA_DeInit (DMA_HandleTypeDef* hdma) {

    if (HAL_DMA_DeInit_Mock != NULL) {
        return HAL_DMA_DeInit_Mock (hdma);
    }

    return HAL_OK;
}

HAL_StatusTypeDef
HAL_DMA_Start (DMA_HandleTypeDef* hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength) {

    if (HAL_DMA_Start_Mock != NULL) {
        return HAL_DMA_Start_Mock (hdma, SrcAddress, DstAddress, DataLength);
    }

    return HAL_OK;
}

HAL_StatusTypeDef
HAL_DMA_Start_IT (DMA_HandleTypeDef* hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength) {

    if (HAL_DMA_Start_IT_Mock != NULL) {
        return HAL_DMA_Start_IT_Mock (hdma, SrcAddress, DstAddress, DataLength);
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_DMA_Abort (DMA_HandleTypeDef* hdma) {

    if (HAL_DMA_Abort_Mock != NULL) {
        return HAL_DMA_Abort_Mock (hdma);
    }

    return HAL_OK;
}

uint32_t HAL_DMA_GetError (const DMA_HandleTypeDef* hdma) {

    if (HAL_DMA_GetError_Mock != NULL) {
        return HAL_DMA_GetError_Mock (hdma);
    }

    return 0;
}

void HAL_GPIO_Init (GPIO_TypeDef* GPIOx, const GPIO_InitTypeDef* GPIO_Init) {

    if (HAL_GPIO_Init_Mock != NULL) {
        HAL_GPIO_Init_Mock (GPIOx, GPIO_Init);
        return;
    }
}

void HAL_GPIO_DeInit (GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin) {

    if (HAL_GPIO_DeInit_Mock != NULL) {
        HAL_GPIO_DeInit_Mock (GPIOx, GPIO_Pin);
        return;
    }
}

void HAL_GPIO_WritePin (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t PinState) {

    if (HAL_GPIO_WritePin_Mock != NULL) {
        HAL_GPIO_WritePin_Mock (GPIOx, GPIO_Pin, PinState);
        return;
    }
}

void HAL_GPIO_TogglePin (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {

    if (HAL_GPIO_TogglePin_Mock != NULL) {
        HAL_GPIO_TogglePin_Mock (GPIOx, GPIO_Pin);
        return;
    }
}

// Timer HAL functions
HAL_StatusTypeDef HAL_TIM_Base_Init (TIM_HandleTypeDef* htim) {
    (void)htim;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_DMA_RegisterCallback (DMA_HandleTypeDef* hdma,
                                             HAL_DMA_CallbackIDTypeDef CallbackID,
                                             void (*pCallback) (DMA_HandleTypeDef* _hdma)) {
    (void)hdma;
    (void)CallbackID;
    (void)pCallback;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Init (TIM_HandleTypeDef* htim) {

    if (HAL_TIM_PWM_Init_Mock != NULL) {
        return HAL_TIM_PWM_Init_Mock (htim);
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_DeInit (TIM_HandleTypeDef* htim) {
    return HAL_OK;
}

HAL_StatusTypeDef
HAL_TIM_PWM_ConfigChannel (TIM_HandleTypeDef* htim, TIM_OC_InitTypeDef* sConfig, uint32_t Channel) {

    if (HAL_TIM_PWM_ConfigChannel_Mock != NULL) {
        return HAL_TIM_PWM_ConfigChannel_Mock (htim, sConfig, Channel);
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start (TIM_HandleTypeDef* htim, uint32_t Channel) {

    if (HAL_TIM_PWM_Start_Mock != NULL) {
        return HAL_TIM_PWM_Start_Mock (htim, Channel);
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop (TIM_HandleTypeDef* htim, uint32_t Channel) {

    if (HAL_TIM_PWM_Stop_Mock != NULL) {
        return HAL_TIM_PWM_Stop_Mock (htim, Channel);
    }

    return HAL_OK;
}

HAL_StatusTypeDef
HAL_TIM_PWM_Start_DMA (TIM_HandleTypeDef* htim, uint32_t Channel, uint32_t* pData, uint16_t Length) {

    if (HAL_TIM_PWM_Start_DMA_Mock != NULL) {
        return HAL_TIM_PWM_Start_DMA_Mock (htim, Channel, pData, Length);
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA (TIM_HandleTypeDef* htim, uint32_t Channel) {

    if (HAL_TIM_PWM_Stop_DMA_Mock != NULL) {
        return HAL_TIM_PWM_Stop_DMA_Mock (htim, Channel);
    }

    return HAL_OK;
}

// void HAL_TIM_ErrorCallback (TIM_HandleTypeDef* htim) {

//     if (HAL_TIM_ErrorCallback_Mock != NULL) {
//         HAL_TIM_ErrorCallback_Mock (htim);
//         return;
//     }

//     // Default weak implementation - do nothing
// }

// void HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef* htim) {

//     if (HAL_TIM_PWM_PulseFinishedCallback_Mock != NULL) {
//         HAL_TIM_PWM_PulseFinishedCallback_Mock (htim);
//         return;
//     }

//     // Default weak implementation - do nothing
// }

// void HAL_TIM_PWM_PulseFinishedHalfCpltCallback (TIM_HandleTypeDef* htim) {

//     if (HAL_TIM_PWM_PulseFinishedHalfCpltCallback_Mock != NULL) {
//         HAL_TIM_PWM_PulseFinishedHalfCpltCallback_Mock (htim);
//         return;
//     }

//     // Default weak implementation - do nothing
// }

void HAL_NVIC_SetPriority (uint32_t IRQn, uint32_t priorityGroup, uint32_t subPriority) {

    if (HAL_NVIC_SetPriority_Mock != NULL) {
        HAL_NVIC_SetPriority_Mock ((IRQn_Type)IRQn, priorityGroup, subPriority);
        return;
    }

    // Default weak implementation - do nothing
}

void HAL_NVIC_EnableIRQ (uint32_t IRQn) {

    if (HAL_NVIC_EnableIRQ_Mock != NULL) {
        HAL_NVIC_EnableIRQ_Mock ((IRQn_Type)IRQn);
        return;
    }

    // Default weak implementation - do nothing
}

// SPI HAL functions
HAL_StatusTypeDef HAL_SPI_Init (SPI_HandleTypeDef* hspi) {

    if (HAL_SPI_Init_Mock != NULL) {
        return HAL_SPI_Init_Mock (hspi);
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_DeInit (SPI_HandleTypeDef* hspi) {

    if (HAL_SPI_DeInit_Mock != NULL) {
        return HAL_SPI_DeInit_Mock (hspi);
    }

    return HAL_OK;
}

HAL_StatusTypeDef
HAL_SPI_Transmit (SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t Size, uint32_t Timeout) {

    if (HAL_SPI_Transmit_Mock != NULL) {
        return HAL_SPI_Transmit_Mock (hspi, pData, Size, Timeout);
    }

    return HAL_OK;
}

HAL_StatusTypeDef
HAL_SPI_Receive (SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t Size, uint32_t Timeout) {

    if (HAL_SPI_Receive_Mock != NULL) {
        return HAL_SPI_Receive_Mock (hspi, pData, Size, Timeout);
    }

    return HAL_OK;
}

HAL_StatusTypeDef
HAL_SPI_TransmitReceive (SPI_HandleTypeDef* hspi, const uint8_t* pTxData, uint8_t* pRxData, uint16_t Size, uint32_t Timeout) {

    if (HAL_SPI_TransmitReceive_Mock != NULL) {
        return HAL_SPI_TransmitReceive_Mock (hspi, pTxData, pRxData, Size, Timeout);
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig (RCC_PeriphCLKInitTypeDef* PeriphClkInit) {

    if (HAL_RCCEx_PeriphCLKConfig_Mock != NULL) {
        return HAL_RCCEx_PeriphCLKConfig_Mock (PeriphClkInit);
    }

    return HAL_OK;
}

// UART HAL functions
HAL_StatusTypeDef HAL_UART_Init (UART_HandleTypeDef* huart) {

    if (HAL_UART_Init_Mock != NULL) {
        return HAL_UART_Init_Mock (huart);
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_DeInit (UART_HandleTypeDef* huart) {

    if (HAL_UART_DeInit_Mock != NULL) {
        return HAL_UART_DeInit_Mock (huart);
    }

    return HAL_OK;
}

HAL_StatusTypeDef
HAL_UART_Transmit (UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size, uint32_t Timeout) {

    if (HAL_UART_Transmit_Mock != NULL) {
        return HAL_UART_Transmit_Mock (huart, pData, Size, Timeout);
    }

    return HAL_OK;
}

HAL_StatusTypeDef
HAL_UART_Receive (UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size, uint32_t Timeout) {

    if (HAL_UART_Receive_Mock != NULL) {
        return HAL_UART_Receive_Mock (huart, pData, Size, Timeout);
    }

    return HAL_OK;
}

HAL_StatusTypeDef
HAL_UART_Receive_IT (UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size) {

    if (HAL_UART_Receive_IT_Mock != NULL) {
        return HAL_UART_Receive_IT_Mock (huart, pData, Size);
    }

    return HAL_OK;
}

void HAL_UART_IRQHandler (UART_HandleTypeDef* huart) {

    if (HAL_UART_IRQHandler_Mock != NULL) {
        HAL_UART_IRQHandler_Mock (huart);
        return;
    }

    // Default weak implementation - do nothing
}

HAL_StatusTypeDef
HAL_UARTEx_SetTxFifoThreshold (UART_HandleTypeDef* huart, uint32_t Threshold) {

    if (HAL_UARTEx_SetTxFifoThreshold_Mock != NULL) {
        return HAL_UARTEx_SetTxFifoThreshold_Mock (huart, Threshold);
    }

    return HAL_OK;
}

HAL_StatusTypeDef
HAL_UARTEx_SetRxFifoThreshold (UART_HandleTypeDef* huart, uint32_t Threshold) {

    if (HAL_UARTEx_SetRxFifoThreshold_Mock != NULL) {
        return HAL_UARTEx_SetRxFifoThreshold_Mock (huart, Threshold);
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_UARTEx_DisableFifoMode (UART_HandleTypeDef* huart) {

    if (HAL_UARTEx_DisableFifoMode_Mock != NULL) {
        return HAL_UARTEx_DisableFifoMode_Mock (huart);
    }

    return HAL_OK;
}

// void HAL_UART_TxCpltCallback (UART_HandleTypeDef* huart) {

//     if (HAL_UART_TxCpltCallback_Mock != NULL) {
//         HAL_UART_TxCpltCallback_Mock (huart);
//         return;
//     }

//     // Default weak implementation - do nothing
// }

// void HAL_UART_RxCpltCallback (UART_HandleTypeDef* huart) {

//     if (HAL_UART_RxCpltCallback_Mock != NULL) {
//         HAL_UART_RxCpltCallback_Mock (huart);
//         return;
//     }

//     // Default weak implementation - do nothing
// }

// void HAL_UART_ErrorCallback (UART_HandleTypeDef* huart) {

//     if (HAL_UART_ErrorCallback_Mock != NULL) {
//         HAL_UART_ErrorCallback_Mock (huart);
//         return;
//     }

//     // Default weak implementation - do nothing
// }