#include "coms/uart.h"
#include "common.h"
#include "hal.h"


#define STLINK_TX_Pin       GPIO_PIN_10
#define STLINK_TX_GPIO_Port GPIOA
#define STLINK_RX_Pin       GPIO_PIN_9
#define STLINK_RX_GPIO_Port GPIOA

// Define extern variables
UART_HandleTypeDef egHandleUSART_1 = { 0 };

/*
 * Below are functions declared by the HAL and defined here.
 */

void USART1_IRQHandler (void) {
    HAL_UART_IRQHandler (&egHandleUSART_1);
}

/*
 * Called by HAL_UART_Init()
 */
void HAL_UART_MspInit (UART_HandleTypeDef* huart) {
    GPIO_InitTypeDef GPIO_InitStruct             = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };
    if (huart->Instance == USART1) {
        /*
         *  Initializes the peripherals clock
         */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
        PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
        if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct) != HAL_OK) {
            CriticalErrorHandler ();
        }

        /* Peripheral clock enable */
        __HAL_RCC_USART1_CLK_ENABLE ();

        __HAL_RCC_GPIOA_CLK_ENABLE ();
        /**USART1 GPIO Configuration
        PA10     ------> USART1_RX
        PA9     ------> USART1_TX
        */
        GPIO_InitStruct.Pin       = STLINK_TX_Pin | STLINK_RX_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
    }
}

void HAL_UART_MspDeInit (UART_HandleTypeDef* huart) {
    if (huart->Instance == USART1) {
        /* Peripheral clock disable */
        __HAL_RCC_USART1_CLK_DISABLE ();

        /**USART1 GPIO Configuration
        PA10     ------> USART1_RX
        PA9     ------> USART1_TX
        */
        HAL_GPIO_DeInit (GPIOA, STLINK_TX_Pin | STLINK_RX_Pin);
    }
}

STATUS_TYPE UARTSystemInit (void) {
    HAL_NVIC_SetPriority (USART1_IRQn, 5, 5);
    HAL_NVIC_EnableIRQ (USART1_IRQn);

    return eSTATUS_SUCCESS;
}