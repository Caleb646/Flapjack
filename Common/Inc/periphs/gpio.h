#ifndef PERIPHS_GPIO_H
#define PERIPHS_GPIO_H
#include "common.h"
#include "hal.h"

#ifndef UNIT_TEST

#define IMU_INT_GPIO_Pin                GPIO_PIN_7
#define IMU_INT_GPIO_Port               GPIOC
#define IMU_INT_EXTI_IRQn               EXTI9_5_IRQn
#define IMU_INT_RCC_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOC_CLK_ENABLE ()

#define STLINK_TX_GPIO_Pin              GPIO_PIN_10
#define STLINK_TX_GPIO_Port             GPIOA
#define STLINK_RX_GPIO_Pin              GPIO_PIN_9
#define STLINK_RX_GPIO_Port             GPIOA

/* TIM8 GPIO Configuration
 * PC6     ------> TIM8_CH1
 * PJ7     ------> TIM8_CH2N
 */
#define TIM8_CH1_GPIO_Pin               GPIO_PIN_6
#define TIM8_CH1_GPIO_Port              GPIOC
#define TIM8_CH1_RCC_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOC_CLK_ENABLE ()
#define TIM8_CH2N_GPIO_Pin              GPIO_PIN_7
#define TIM8_CH2N_GPIO_Port             GPIOJ
#define TIM8_CH2N_RCC_GPIO_CLK_ENABLE() __HAL_RCC_GPIOJ_CLK_ENABLE ()

/* TIM13 GPIO Configuration
 * PF8     ------> TIM13_CH1
 */
#define TIM13_CH1_GPIO_Pin              GPIO_PIN_8
#define TIM13_CH1_GPIO_Port             GPIOF
#define TIM13_CH1_RCC_GPIO_CLK_ENABLE() __HAL_RCC_GPIOF_CLK_ENABLE ()

#else

#define IMU_INT_GPIO_Pin
#define IMU_INT_GPIO_Port
#define IMU_INT_EXTI_IRQn
#define IMU_INT_RCC_GPIO_CLK_ENABLE()

#define STLINK_TX_GPIO_Pin
#define STLINK_TX_GPIO_Port
#define STLINK_RX_GPIO_Pin
#define STLINK_RX_GPIO_Port

/* TIM8 GPIO Configuration
 * PC6     ------> TIM8_CH1
 * PJ7     ------> TIM8_CH2N
 */
#define TIM8_CH1_GPIO_Pin
#define TIM8_CH1_GPIO_Port
#define TIM8_CH1_RCC_GPIO_CLK_ENABLE()
#define TIM8_CH2N_GPIO_Pin
#define TIM8_CH2N_GPIO_Port
#define TIM8_CH2N_RCC_GPIO_CLK_ENABLE()

/* TIM13 GPIO Configuration
 * PF8     ------> TIM13_CH1
 */
#define TIM13_CH1_GPIO_Pin
#define TIM13_CH1_GPIO_Port
#define TIM13_CH1_RCC_GPIO_CLK_ENABLE()

#endif


#endif // PERIPHS_GPIO_H