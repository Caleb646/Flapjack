/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file         stm32h7xx_hal_msp.c
 * @brief        This file provides code for the MSP Initialization
 *               and de-Initialization codes.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "hal.h"

#include "core/core.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

#define OSC32_OUT_Pin            GPIO_PIN_15
#define OSC32_OUT_GPIO_Port      GPIOC
#define OSC32_IN_Pin             GPIO_PIN_14
#define OSC32_IN_GPIO_Port       GPIOC
// #define STLINK_TX_GPIO_Pin GPIO_PIN_10
// #define STLINK_TX_GPIO_Port GPIOA
// #define STLINK_RX_GPIO_Pin GPIO_PIN_9
// #define STLINK_RX_GPIO_Port GPIOA
#define CEC_CK_MCO1_Pin          GPIO_PIN_8
#define CEC_CK_MCO1_GPIO_Port    GPIOA
#define SPI2_SCK_Pin             GPIO_PIN_12
#define SPI2_SCK_GPIO_Port       GPIOA
#define SPI2_NSS_Pin             GPIO_PIN_11
#define SPI2_NSS_GPIO_Port       GPIOA
// #define IMU_INT_GPIO_Pin              GPIO_PIN_7
// #define IMU_INT_GPIO_Port        GPIOC
// #define IMU_INT_EXTI_IRQn        EXTI9_5_IRQn
#define OSC_OUT_Pin              GPIO_PIN_1
#define OSC_OUT_GPIO_Port        GPIOH
#define OSC_IN_Pin               GPIO_PIN_0
#define OSC_IN_GPIO_Port         GPIOH
#define PMOD_14_ARD_D3_Pin       GPIO_PIN_8
#define PMOD_14_ARD_D3_GPIO_Port GPIOF
#define PMOD_3_Pin               GPIO_PIN_2
#define PMOD_3_GPIO_Port         GPIOC
#define PMOD_2_Pin               GPIO_PIN_3
#define PMOD_2_GPIO_Port         GPIOC
#define ARD_D6_Pin               GPIO_PIN_7
#define ARD_D6_GPIO_Port         GPIOJ
#define ARD_D9_Pin               GPIO_PIN_6
#define ARD_D9_GPIO_Port         GPIOJ

void HAL_TIM_MspPostInit (TIM_HandleTypeDef* htim);

/**
 * @brief SPI MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspInit (SPI_HandleTypeDef* hspi) {
    GPIO_InitTypeDef GPIO_InitStruct             = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };
    if (hspi->Instance == SPI2) {
        /* USER CODE BEGIN SPI2_MspInit 0 */

        /* USER CODE END SPI2_MspInit 0 */

        /** Initializes the peripherals clock
         */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
        PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
        if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct) != HAL_OK) {
            CriticalErrorHandler ();
        }

        /* Peripheral clock enable */
        __HAL_RCC_SPI2_CLK_ENABLE ();

        __HAL_RCC_GPIOA_CLK_ENABLE ();
        __HAL_RCC_GPIOC_CLK_ENABLE ();
        /**SPI2 GPIO Configuration
        PA12     ------> SPI2_SCK
        PA11     ------> SPI2_NSS
        PC2     ------> SPI2_MISO
        PC3     ------> SPI2_MOSI
        */
        GPIO_InitStruct.Pin       = SPI2_SCK_Pin | SPI2_NSS_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = PMOD_3_Pin | PMOD_2_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init (GPIOC, &GPIO_InitStruct);

        /* USER CODE BEGIN SPI2_MspInit 1 */

        /* USER CODE END SPI2_MspInit 1 */
    }
}

/**
 * @brief SPI MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspDeInit (SPI_HandleTypeDef* hspi) {
    if (hspi->Instance == SPI2) {
        /* USER CODE BEGIN SPI2_MspDeInit 0 */

        /* USER CODE END SPI2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_SPI2_CLK_DISABLE ();

        /**SPI2 GPIO Configuration
        PA12     ------> SPI2_SCK
        PA11     ------> SPI2_NSS
        PC2     ------> SPI2_MISO
        PC3     ------> SPI2_MOSI
        */
        HAL_GPIO_DeInit (GPIOA, SPI2_SCK_Pin | SPI2_NSS_Pin);

        HAL_GPIO_DeInit (GPIOC, PMOD_3_Pin | PMOD_2_Pin);

        /* USER CODE BEGIN SPI2_MspDeInit 1 */

        /* USER CODE END SPI2_MspDeInit 1 */
    }
}

/**
 * @brief TIM_Base MSP Initialization
 * This function configures the hardware resources used in this example
 * @param htim_base: TIM_Base handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspInit (TIM_HandleTypeDef* htim_base) {
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    if (htim_base->Instance == TIM8) {
        /* USER CODE BEGIN TIM8_MspInit 0 */

        /* USER CODE END TIM8_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM8_CLK_ENABLE ();

        __HAL_RCC_GPIOJ_CLK_ENABLE ();
        /**TIM8 GPIO Configuration
        PJ6     ------> TIM8_CH2
        */
        GPIO_InitStruct.Pin       = ARD_D9_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init (ARD_D9_GPIO_Port, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM8_MspInit 1 */

        /* USER CODE END TIM8_MspInit 1 */
    } else if (htim_base->Instance == TIM13) {
        /* USER CODE BEGIN TIM13_MspInit 0 */

        /* USER CODE END TIM13_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM13_CLK_ENABLE ();
        /* USER CODE BEGIN TIM13_MspInit 1 */

        /* USER CODE END TIM13_MspInit 1 */
    }
}

void HAL_TIM_MspPostInit (TIM_HandleTypeDef* htim) {
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    if (htim->Instance == TIM8) {
        /* USER CODE BEGIN TIM8_MspPostInit 0 */

        /* USER CODE END TIM8_MspPostInit 0 */
        __HAL_RCC_GPIOC_CLK_ENABLE ();
        __HAL_RCC_GPIOJ_CLK_ENABLE ();
        /**TIM8 GPIO Configuration
        PC6     ------> TIM8_CH1
        PJ7     ------> TIM8_CH2N
        */
        GPIO_InitStruct.Pin       = GPIO_PIN_6;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init (GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = ARD_D6_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init (ARD_D6_GPIO_Port, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM8_MspPostInit 1 */

        /* USER CODE END TIM8_MspPostInit 1 */
    } else if (htim->Instance == TIM13) {
        /* USER CODE BEGIN TIM13_MspPostInit 0 */

        /* USER CODE END TIM13_MspPostInit 0 */

        __HAL_RCC_GPIOF_CLK_ENABLE ();
        /**TIM13 GPIO Configuration
        PF8     ------> TIM13_CH1
        */
        GPIO_InitStruct.Pin       = PMOD_14_ARD_D3_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF9_TIM13;
        HAL_GPIO_Init (PMOD_14_ARD_D3_GPIO_Port, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM13_MspPostInit 1 */

        /* USER CODE END TIM13_MspPostInit 1 */
    }
}
/**
 * @brief TIM_Base MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param htim_base: TIM_Base handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspDeInit (TIM_HandleTypeDef* htim_base) {
    if (htim_base->Instance == TIM8) {
        /* USER CODE BEGIN TIM8_MspDeInit 0 */

        /* USER CODE END TIM8_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM8_CLK_DISABLE ();

        /**TIM8 GPIO Configuration
        PC6     ------> TIM8_CH1
        PJ7     ------> TIM8_CH2N
        PJ6     ------> TIM8_CH2
        */
        HAL_GPIO_DeInit (GPIOC, GPIO_PIN_6);

        HAL_GPIO_DeInit (GPIOJ, ARD_D6_Pin | ARD_D9_Pin);

        /* USER CODE BEGIN TIM8_MspDeInit 1 */

        /* USER CODE END TIM8_MspDeInit 1 */
    } else if (htim_base->Instance == TIM13) {
        /* USER CODE BEGIN TIM13_MspDeInit 0 */

        /* USER CODE END TIM13_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM13_CLK_DISABLE ();
        /* USER CODE BEGIN TIM13_MspDeInit 1 */

        /* USER CODE END TIM13_MspDeInit 1 */
    }
}

/**
 * @brief UART MSP Initialization
 * This function configures the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
// void HAL_UART_MspInit(UART_HandleTypeDef* huart)
// {
//   GPIO_InitTypeDef GPIO_InitStruct = {0};
//   RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
//   if(huart->Instance==USART1)
//   {
//     /* USER CODE BEGIN USART1_MspInit 0 */

//     /* USER CODE END USART1_MspInit 0 */

//   /** Initializes the peripherals clock
//   */
//     PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
//     PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
//     if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
//     {
//       Error_Handler();
//     }

//     /* Peripheral clock enable */
//     __HAL_RCC_USART1_CLK_ENABLE();

//     __HAL_RCC_GPIOA_CLK_ENABLE();
//     /**USART1 GPIO Configuration
//     PA10     ------> USART1_RX
//     PA9     ------> USART1_TX
//     */
//     GPIO_InitStruct.Pin = STLINK_TX_GPIO_Pin|STLINK_RX_GPIO_Pin;
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//     GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
//     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//     /* USER CODE BEGIN USART1_MspInit 1 */

//     /* USER CODE END USART1_MspInit 1 */

//   }

// }

/**
 * @brief UART MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
// void HAL_UART_MspDeInit (UART_HandleTypeDef* huart) {
//     if (huart->Instance == USART1) {
//         /* USER CODE BEGIN USART1_MspDeInit 0 */

//         /* USER CODE END USART1_MspDeInit 0 */
//         /* Peripheral clock disable */
//         __HAL_RCC_USART1_CLK_DISABLE ();

//         /**USART1 GPIO Configuration
//         PA10     ------> USART1_RX
//         PA9     ------> USART1_TX
//         */
//         HAL_GPIO_DeInit (GPIOA, STLINK_TX_GPIO_Pin | STLINK_RX_GPIO_Pin);

//         /* USER CODE BEGIN USART1_MspDeInit 1 */

//         /* USER CODE END USART1_MspDeInit 1 */
//     }
// }

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
