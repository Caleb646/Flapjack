#include "periphs/uart.h"
#include "common.h"
#include "hal.h"
#include "periphs/gpio.h"


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

        if (GPIOInitUart (huart->Instance) != eSTATUS_SUCCESS) {
            CriticalErrorHandler ();
        }
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
        // HAL_GPIO_DeInit (GPIOA, STLINK_TX_GPIO_Pin | STLINK_RX_GPIO_Pin);
    }
}

eSTATUS_t UARTSystemInit (void) {

    return eSTATUS_SUCCESS;
}

eSTATUS_t UARTSystemEnableInterrupts (void) {
    HAL_NVIC_SetPriority (USART1_IRQn, 10, 10);
    HAL_NVIC_EnableIRQ (USART1_IRQn);

    return eSTATUS_SUCCESS;
}