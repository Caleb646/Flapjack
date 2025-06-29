/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit (TIM_HandleTypeDef* htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler (void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OSC32_OUT_Pin            GPIO_PIN_15
#define OSC32_OUT_GPIO_Port      GPIOC
#define OSC32_IN_Pin             GPIO_PIN_14
#define OSC32_IN_GPIO_Port       GPIOC
// #define STLINK_TX_Pin GPIO_PIN_10
// #define STLINK_TX_GPIO_Port GPIOA
// #define STLINK_RX_Pin GPIO_PIN_9
// #define STLINK_RX_GPIO_Port GPIOA
#define CEC_CK_MCO1_Pin          GPIO_PIN_8
#define CEC_CK_MCO1_GPIO_Port    GPIOA
#define SPI2_SCK_Pin             GPIO_PIN_12
#define SPI2_SCK_GPIO_Port       GPIOA
#define SPI2_NSS_Pin             GPIO_PIN_11
#define SPI2_NSS_GPIO_Port       GPIOA
#define IMU_INT_Pin              GPIO_PIN_7
#define IMU_INT_GPIO_Port        GPIOC
#define IMU_INT_EXTI_IRQn        EXTI9_5_IRQn
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

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
