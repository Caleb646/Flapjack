#ifndef PERIPHS_GPIO_H
#define PERIPHS_GPIO_H

#include "common.h"
#include "hal.h"

#define IMU_INT_EXTI_IRQn           EXTI9_5_IRQn

#define TIM5_CH1_GPIO_Pin           GPIO_PIN_0
#define TIM5_CH1_GPIO_Port          GPIOA
#define TIM5_CH2_GPIO_Pin           GPIO_PIN_1
#define TIM5_CH2_GPIO_Port          GPIOA
#define TIM5_CH3_GPIO_Pin           GPIO_PIN_2
#define TIM5_CH3_GPIO_Port          GPIOA
#define TIM5_CH4_GPIO_Pin           GPIO_PIN_3
#define TIM5_CH4_GPIO_Port          GPIOA
#define TIM5_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE ()

#define TIM8_CH1_GPIO_Pin           GPIO_PIN_6
#define TIM8_CH1_GPIO_Port          GPIOC
#define TIM8_CH2_GPIO_Pin           GPIO_PIN_7
#define TIM8_CH2_GPIO_Port          GPIOC
#define TIM8_CH3_GPIO_Pin           GPIO_PIN_8
#define TIM8_CH3_GPIO_Port          GPIOC
#define TIM8_CH4_GPIO_Pin           GPIO_PIN_9
#define TIM8_CH4_GPIO_Port          GPIOC
#define TIM8_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE ()

#define TIM12_CH1_GPIO_Pin          GPIO_PIN_6
#define TIM12_CH1_GPIO_Port         GPIOH
#define TIM12_CH2_GPIO_Pin          GPIO_PIN_9
#define TIM12_CH2_GPIO_Port         GPIOH
#define TIM12_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOH_CLK_ENABLE ()

/*
 *
 * SPI GPIO Configuration
 *
 */

// SPI_1: SCK, MISO, MOSI
#define SPI1_DATA_Pins              (GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7)
#define SPI1_DATA_GPIO_Port         GPIOA
#define SPI1_DATA_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE ()
// NSS
#define SPI1_NSS_GPIO_Pin           GPIO_PIN_4
#define SPI1_NSS_GPIO_Port          GPIOC
#define SPI1_NSS_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOC_CLK_ENABLE ()

// SPI_3: SCK, MISO, MOSI
#define SPI3_DATA_Pins              (GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12)
#define SPI3_DATA_GPIO_Port         GPIOC
#define SPI3_DATA_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE ()
// NSS
#define SPI3_NSS_GPIO_Pin           GPIO_PIN_15
#define SPI3_NSS_GPIO_Port          GPIOA
#define SPI3_NSS_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE ()

// SPI_5: SCK, MISO, MOSI
#define SPI5_DATA_Pins              (GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9)
#define SPI5_DATA_GPIO_Port         GPIOF
#define SPI5_DATA_GPIO_CLK_ENABLE() __HAL_RCC_GPIOF_CLK_ENABLE ()
// NSS
#define SPI5_NSS_GPIO_Pin           GPIO_PIN_10
#define SPI5_NSS_GPIO_Port          GPIOF
#define SPI5_NSS_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOF_CLK_ENABLE ()

/*
 *
 * EXTI Interrupts GPIO Configuration
 *
 */

#define EXTI5_GPIO_Pin              GPIO_PIN_5
#define EXTI5_GPIO_Port             GPIOC
#define EXTI5_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE ()

#define EXTI6_GPIO_Pin              GPIO_PIN_6
#define EXTI6_GPIO_Port             GPIOF
#define EXTI6_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOF_CLK_ENABLE ()

#define EXTI3_GPIO_Pin              GPIO_PIN_3
#define EXTI3_GPIO_Port             GPIOF
#define EXTI3_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOF_CLK_ENABLE ()

/*
 *
 * UART GPIO Configuration
 *
 */

// UART1: RX, TX
// #define STLINK_RX_GPIO_Pin          GPIO_PIN_9
// #define STLINK_RX_GPIO_Port         GPIOA
// #define STLINK_TX_GPIO_Pin          GPIO_PIN_10
// #define STLINK_TX_GPIO_Port         GPIOA

// UART1: RX, TX
#define UART1_RX_GPIO_Pin           GPIO_PIN_15
#define UART1_TX_GPIO_Pin           GPIO_PIN_14
#define UART1_GPIO_Port             GPIOB
#define UART1_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE ()
// #define UART1_GPIO_AF          GPIO_AF7_USART1

// UART2: RX, TX
#define UART2_RX_GPIO_Pin           GPIO_PIN_6
#define UART2_TX_GPIO_Pin           GPIO_PIN_5
#define UART2_GPIO_Port             GPIOD
#define UART2_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOD_CLK_ENABLE ()
// #define UART2_GPIO_AF             GPIO_AF7_USART2

// UART3: RX, TX
#define UART3_RX_GPIO_Pin           GPIO_PIN_11
#define UART3_TX_GPIO_Pin           GPIO_PIN_10
#define UART3_GPIO_Port             GPIOB
#define UART3_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE ()
// #define UART3_GPIO_AF            GPIO_AF7_USART3

/*
 *
 * I2C GPIO Configuration
 *
 */

// Current Sensor I2C: SCL, SDA
#define I2C1_SCL_GPIO_Pin           GPIO_PIN_6
#define I2C1_SDA_GPIO_Pin           GPIO_PIN_7
#define I2C1_GPIO_Port              GPIOB
#define I2C1_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE ()

typedef struct {
    GPIO_TypeDef* pNSSPort;
    uint16_t nssPin;
} GPIOSPI_t;

typedef struct {
    GPIO_TypeDef* pSCLPort;
    GPIO_TypeDef* pSDAPort;
    uint16_t sclPin;
    uint16_t sdaPin;
} GPIOI2C_t;

typedef struct {
    GPIO_TypeDef* pTXPort;
    GPIO_TypeDef* pRXPort;
    uint16_t txPin;
    uint16_t rxPin;
} GPIOUART_t;

typedef struct {
    GPIO_TypeDef* pPort;
    uint16_t pin;
} GPIOEXTI_t;

typedef struct {
    GPIO_TypeDef* pPort;
    uint16_t pin;
} GPIOTimer_t;

eSTATUS_t GPIOInitUART (USART_TypeDef* pInstance, GPIOUART_t* pOutGPIO);
eSTATUS_t GPIOInitSPI (SPI_TypeDef* pInstance, GPIOSPI_t* pOutGPIO);
eSTATUS_t GPIOInitEXTI (IRQn_Type irq, uint32_t pin, GPIOEXTI_t* pOutGPIO);
eSTATUS_t GPIOInitTimer (TIM_TypeDef* pInstance, uint32_t channelId, GPIOTimer_t* pOutGPIO);
eSTATUS_t GPIOInitI2C (I2C_TypeDef* pInstance, GPIOI2C_t* pOutGPIO);

#endif // PERIPHS_GPIO_H