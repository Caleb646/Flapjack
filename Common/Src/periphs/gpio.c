#include "periphs/gpio.h"
#include "common.h"

#define TIM5_CH1_GPIO_Pin           GPIO_PIN_0
#define TIM5_CH1_GPIO_Port          GPIOA
#define TIM5_CH2_GPIO_Pin           GPIO_PIN_1
#define TIM5_CH2_GPIO_Port          GPIOA
#define TIM5_CH3_GPIO_Pin           GPIO_PIN_2
#define TIM5_CH3_GPIO_Port          GPIOA
#define TIM5_CH4_GPIO_Pin           GPIO_PIN_3
#define TIM5_CH4_GPIO_Port          GPIOA

#define TIM8_CH1_GPIO_Pin           GPIO_PIN_6
#define TIM8_CH1_GPIO_Port          GPIOC
#define TIM8_CH2_GPIO_Pin           GPIO_PIN_7
#define TIM8_CH2_GPIO_Port          GPIOC
#define TIM8_CH3_GPIO_Pin           GPIO_PIN_8
#define TIM8_CH3_GPIO_Port          GPIOC
#define TIM8_CH4_GPIO_Pin           GPIO_PIN_9
#define TIM8_CH4_GPIO_Port          GPIOC

#define TIM12_CH1_GPIO_Pin          GPIO_PIN_6
#define TIM12_CH1_GPIO_Port         GPIOH
#define TIM12_CH2_GPIO_Pin          GPIO_PIN_9
#define TIM12_CH2_GPIO_Port         GPIOH

/*
 *
 * SPI GPIO Configuration
 *
 */

// SPI_1: SCK, MISO, MOSI
#define SPI1_DATA_Pins              (GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7)
#define SPI1_DATA_GPIO_Port         GPIOA
// NSS
#define SPI1_NSS_GPIO_Pin           GPIO_PIN_4
#define SPI1_NSS_GPIO_Port          GPIOC

// SPI_3: SCK, MISO, MOSI
#define SPI3_DATA_Pins              (GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12)
#define SPI3_DATA_GPIO_Port         GPIOC
// NSS
#define SPI3_NSS_GPIO_Pin           GPIO_PIN_15
#define SPI3_NSS_GPIO_Port          GPIOA

// SPI_5: SCK, MISO, MOSI
#define SPI5_DATA_Pins              (GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9)
#define SPI5_DATA_GPIO_Port         GPIOF
// NSS
#define SPI5_NSS_GPIO_Pin           GPIO_PIN_10
#define SPI5_NSS_GPIO_Port          GPIOF

/*
 *
 * EXTI Interrupts GPIO Configuration
 *
 */

// IMU Interrupt Pin
#define IMU_INT_GPIO_Pin            GPIO_PIN_7
#define IMU_INT_GPIO_Port           GPIOC

// Barometer Interrupt Pin
#define BARO_INT_GPIO_Pin           GPIO_PIN_10
#define BARO_INT_GPIO_Port          GPIOF

// Magnetometer Interrupt Pin
#define MAG_INT_GPIO_Pin            GPIO_PIN_3
#define MAG_INT_GPIO_Port           GPIOF

/*
 *
 * UART GPIO Configuration
 *
 */

// ST-LINK VCP UART1: RX, TX
#define STLINK_RX_GPIO_Pin          GPIO_PIN_9
#define STLINK_RX_GPIO_Port         GPIOA
#define STLINK_TX_GPIO_Pin          GPIO_PIN_10
#define STLINK_TX_GPIO_Port         GPIOA

// Serial Debug UART1: RX, TX
#define DEBUG_UART_RX_GPIO_Pin      GPIO_PIN_15
#define DEBUG_UART_RX_GPIO_Port     GPIOB
#define DEBUG_UART_TX_GPIO_Pin      GPIO_PIN_14
#define DEBUG_UART_TX_GPIO_Port     GPIOB
#define DEBUG_UART_GPIO_AF          GPIO_AF7_USART1

// RF Receiver UART2: RX, TX
#define RF_UART_RX_GPIO_Pin         GPIO_PIN_6
#define RF_UART_RX_GPIO_Port        GPIOD
#define RF_UART_TX_GPIO_Pin         GPIO_PIN_5
#define RF_UART_TX_GPIO_Port        GPIOD
#define RF_UART_GPIO_AF             GPIO_AF7_USART2

// GPS UART3: RX, TX
#define GPS_UART_RX_GPIO_Pin        GPIO_PIN_11
#define GPS_UART_RX_GPIO_Port       GPIOB
#define GPS_UART_TX_GPIO_Pin        GPIO_PIN_10
#define GPS_UART_TX_GPIO_Port       GPIOB
#define GPS_UART_GPIO_AF            GPIO_AF7_USART3

/*
 *
 * I2C GPIO Configuration
 *
 */

// Current Sensor I2C: SCL, SDA
#define CURR_SENS_I2C_SCL_GPIO_Pin  GPIO_PIN_6
#define CURR_SENS_I2C_SCL_GPIO_Port GPIOB
#define CURR_SENS_I2C_SDA_GPIO_Pin  GPIO_PIN_7
#define CURR_SENS_I2C_SDA_GPIO_Port GPIOB


static eSTATUS_t
GPIOInit_ (GPIO_TypeDef* pPort, uint16_t pin, uint32_t mode, uint32_t pull, uint32_t alt, uint32_t speed) {

    if (pPort == NULL) {
        LOG_ERROR ("Invalid GPIO port");
        return eSTATUS_FAILURE;
    }
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Pin              = pin;
    GPIO_InitStruct.Mode             = mode;
    GPIO_InitStruct.Pull             = pull;
    GPIO_InitStruct.Speed            = speed;
    GPIO_InitStruct.Alternate        = alt;

    HAL_GPIO_Init (pPort, &GPIO_InitStruct);
    return eSTATUS_SUCCESS;
}

eSTATUS_t GPIOInit (void) {
    __HAL_RCC_GPIOA_CLK_ENABLE ();
    __HAL_RCC_GPIOB_CLK_ENABLE ();
    __HAL_RCC_GPIOC_CLK_ENABLE ();
    __HAL_RCC_GPIOD_CLK_ENABLE ();
    __HAL_RCC_GPIOE_CLK_ENABLE ();
    __HAL_RCC_GPIOF_CLK_ENABLE ();
    __HAL_RCC_GPIOG_CLK_ENABLE ();
    __HAL_RCC_GPIOH_CLK_ENABLE ();
    __HAL_RCC_GPIOI_CLK_ENABLE ();
    __HAL_RCC_GPIOJ_CLK_ENABLE ();
    __HAL_RCC_GPIOK_CLK_ENABLE ();

    /********* Motor Timers *******/
    // Left Motor Timer
    GPIOInit_ (TIM12_CH1_GPIO_Port, TIM12_CH1_GPIO_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF2_TIM12, GPIO_SPEED_FREQ_HIGH);
    // Right Motor Timer
    GPIOInit_ (TIM12_CH2_GPIO_Port, TIM12_CH2_GPIO_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF2_TIM12, GPIO_SPEED_FREQ_HIGH);

    /********* Servo Timers *******/
    // Left Servo Timers
    GPIOInit_ (TIM5_CH1_GPIO_Port, TIM5_CH1_GPIO_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF2_TIM5, GPIO_SPEED_FREQ_HIGH);
    GPIOInit_ (TIM5_CH2_GPIO_Port, TIM5_CH2_GPIO_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF2_TIM5, GPIO_SPEED_FREQ_HIGH);
    GPIOInit_ (TIM5_CH3_GPIO_Port, TIM5_CH3_GPIO_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF2_TIM5, GPIO_SPEED_FREQ_HIGH);
    GPIOInit_ (TIM5_CH4_GPIO_Port, TIM5_CH4_GPIO_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF2_TIM5, GPIO_SPEED_FREQ_HIGH);
    // Right Servo Timers
    GPIOInit_ (TIM8_CH1_GPIO_Port, TIM8_CH1_GPIO_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF3_TIM8, GPIO_SPEED_FREQ_HIGH);
    GPIOInit_ (TIM8_CH2_GPIO_Port, TIM8_CH2_GPIO_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF3_TIM8, GPIO_SPEED_FREQ_HIGH);
    GPIOInit_ (TIM8_CH3_GPIO_Port, TIM8_CH3_GPIO_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF3_TIM8, GPIO_SPEED_FREQ_HIGH);
    GPIOInit_ (TIM8_CH4_GPIO_Port, TIM8_CH4_GPIO_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF3_TIM8, GPIO_SPEED_FREQ_HIGH);

    /********* IMU SPI1 & Interrupts *********/
    // IMU SPI Pins
    GPIOInit_ (SPI1_DATA_GPIO_Port, SPI1_DATA_Pins, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI1, GPIO_SPEED_FREQ_HIGH);
    GPIOInit_ (SPI1_NSS_GPIO_Port, SPI1_NSS_GPIO_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI1, GPIO_SPEED_FREQ_HIGH);
    // IMU Interrupt Pin
    GPIOInit_ (IMU_INT_GPIO_Port, IMU_INT_GPIO_Pin, GPIO_MODE_IT_RISING, GPIO_NOPULL, GPIO_AF0_MCO, GPIO_SPEED_FREQ_LOW);

    /********* Flash Storage SPI3 *********/
    // Flash Storage SPI Pins
    GPIOInit_ (SPI3_DATA_GPIO_Port, SPI3_DATA_Pins, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF6_SPI3, GPIO_SPEED_FREQ_HIGH);
    GPIOInit_ (SPI3_NSS_GPIO_Port, SPI3_NSS_GPIO_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF6_SPI3, GPIO_SPEED_FREQ_HIGH);

    /********* Barometer & Magnetometer SPI5 & Interrupts *********/
    // Barometer & Magnetometer SPI Pins
    GPIOInit_ (SPI5_DATA_GPIO_Port, SPI5_DATA_Pins, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI5, GPIO_SPEED_FREQ_HIGH);
    GPIOInit_ (SPI5_NSS_GPIO_Port, SPI5_NSS_GPIO_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI5, GPIO_SPEED_FREQ_HIGH);
    // Barometer & Magnetometer Interrupt Pin
    GPIOInit_ (BARO_INT_GPIO_Port, BARO_INT_GPIO_Pin, GPIO_MODE_IT_RISING, GPIO_NOPULL, GPIO_AF0_MCO, GPIO_SPEED_FREQ_LOW);
    GPIOInit_ (MAG_INT_GPIO_Port, MAG_INT_GPIO_Pin, GPIO_MODE_IT_RISING, GPIO_NOPULL, GPIO_AF0_MCO, GPIO_SPEED_FREQ_LOW);

    /********* Serial Debug *********/
    // UART
    GPIOInit_ (DEBUG_UART_TX_GPIO_Port, DEBUG_UART_TX_GPIO_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, DEBUG_UART_GPIO_AF, GPIO_SPEED_FREQ_HIGH);
    GPIOInit_ (DEBUG_UART_RX_GPIO_Port, DEBUG_UART_RX_GPIO_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, DEBUG_UART_GPIO_AF, GPIO_SPEED_FREQ_HIGH);

    /********* GPS *********/
    // UART
    GPIOInit_ (GPS_UART_TX_GPIO_Port, GPS_UART_TX_GPIO_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPS_UART_GPIO_AF, GPIO_SPEED_FREQ_HIGH);
    GPIOInit_ (GPS_UART_RX_GPIO_Port, GPS_UART_RX_GPIO_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPS_UART_GPIO_AF, GPIO_SPEED_FREQ_HIGH);

    /********* RF Receiver *********/
    // UART
    GPIOInit_ (RF_UART_TX_GPIO_Port, RF_UART_TX_GPIO_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, RF_UART_GPIO_AF, GPIO_SPEED_FREQ_HIGH);
    GPIOInit_ (RF_UART_RX_GPIO_Port, RF_UART_RX_GPIO_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, RF_UART_GPIO_AF, GPIO_SPEED_FREQ_HIGH);

    // /********* ST-LINK VCP *********/
    // GPIOInit_ (STLINK_TX_GPIO_Port, STLINK_TX_GPIO_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF7_USART1, GPIO_SPEED_FREQ_HIGH);
    // GPIOInit_ (STLINK_RX_GPIO_Port, STLINK_RX_GPIO_Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF7_USART1, GPIO_SPEED_FREQ_HIGH);
}