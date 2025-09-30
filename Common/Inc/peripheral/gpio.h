#ifndef PERIPHS_GPIO_H
#define PERIPHS_GPIO_H

#include "common.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "hal.h"

// #define TIM5_CH1_GPIO_Pin           GPIO_PIN_0
// #define TIM5_CH1_GPIO_Port          GPIOA
// #define TIM5_CH2_GPIO_Pin           GPIO_PIN_1
// #define TIM5_CH2_GPIO_Port          GPIOA
// #define TIM5_CH3_GPIO_Pin           GPIO_PIN_2
// #define TIM5_CH3_GPIO_Port          GPIOA
// #define TIM5_CH4_GPIO_Pin           GPIO_PIN_3
// #define TIM5_CH4_GPIO_Port          GPIOA
// #define TIM5_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE ()

// #define TIM8_CH1_GPIO_Pin           GPIO_PIN_6
// #define TIM8_CH1_GPIO_Port          GPIOC
// #define TIM8_CH2_GPIO_Pin           GPIO_PIN_7
// #define TIM8_CH2_GPIO_Port          GPIOC
// #define TIM8_CH3_GPIO_Pin           GPIO_PIN_8
// #define TIM8_CH3_GPIO_Port          GPIOC
// #define TIM8_CH4_GPIO_Pin           GPIO_PIN_9
// #define TIM8_CH4_GPIO_Port          GPIOC
// #define TIM8_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE ()

// #define TIM12_CH1_GPIO_Pin          GPIO_PIN_6
// #define TIM12_CH1_GPIO_Port         GPIOH
// #define TIM12_CH2_GPIO_Pin          GPIO_PIN_9
// #define TIM12_CH2_GPIO_Port         GPIOH
// #define TIM12_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOH_CLK_ENABLE ()

// /*
//  *
//  * SPI GPIO Configuration
//  *
//  */
// // SPI_1: SCK, MISO, MOSI
// #define SPI1_DATA_Pins              (GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7)
// #define SPI1_DATA_GPIO_Port         GPIOA
// #define SPI1_DATA_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE ()
// // NSS
// #define SPI1_NSS_GPIO_Pin           GPIO_PIN_4
// #define SPI1_NSS_GPIO_Port          GPIOC
// #define SPI1_NSS_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOC_CLK_ENABLE ()

// // SPI_3: SCK, MISO, MOSI
// #define SPI3_DATA_Pins              (GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12)
// #define SPI3_DATA_GPIO_Port         GPIOC
// #define SPI3_DATA_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE ()
// // NSS
// #define SPI3_NSS_GPIO_Pin           GPIO_PIN_15
// #define SPI3_NSS_GPIO_Port          GPIOA
// #define SPI3_NSS_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE ()

// // SPI_5: SCK, MISO, MOSI
// #define SPI5_DATA_Pins              (GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9)
// #define SPI5_DATA_GPIO_Port         GPIOF
// #define SPI5_DATA_GPIO_CLK_ENABLE() __HAL_RCC_GPIOF_CLK_ENABLE ()
// // NSS
// #define SPI5_NSS_GPIO_Pin           GPIO_PIN_10
// #define SPI5_NSS_GPIO_Port          GPIOF
// #define SPI5_NSS_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOF_CLK_ENABLE ()

// /*
//  *
//  * EXTI Interrupts GPIO Configuration
//  *
//  */

// #define EXTI5_GPIO_Pin              GPIO_PIN_5
// #define EXTI5_GPIO_Port             GPIOC
// #define EXTI5_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE ()

// #define EXTI6_GPIO_Pin              GPIO_PIN_6
// #define EXTI6_GPIO_Port             GPIOF
// #define EXTI6_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOF_CLK_ENABLE ()

// #define EXTI3_GPIO_Pin              GPIO_PIN_3
// #define EXTI3_GPIO_Port             GPIOF
// #define EXTI3_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOF_CLK_ENABLE ()

// /*
//  *
//  * UART GPIO Configuration
//  *
//  */

// // UART1: RX, TX
// // #define STLINK_RX_GPIO_Pin          GPIO_PIN_9
// // #define STLINK_RX_GPIO_Port         GPIOA
// // #define STLINK_TX_GPIO_Pin          GPIO_PIN_10
// // #define STLINK_TX_GPIO_Port         GPIOA

// // UART1: RX, TX
// #define UART1_RX_GPIO_Pin           GPIO_PIN_15
// #define UART1_TX_GPIO_Pin           GPIO_PIN_14
// #define UART1_GPIO_Port             GPIOB
// #define UART1_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE ()
// // #define UART1_GPIO_AF          GPIO_AF7_USART1

// // UART2: RX, TX
// #define UART2_RX_GPIO_Pin           GPIO_PIN_6
// #define UART2_TX_GPIO_Pin           GPIO_PIN_5
// #define UART2_GPIO_Port             GPIOD
// #define UART2_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOD_CLK_ENABLE ()
// // #define UART2_GPIO_AF             GPIO_AF7_USART2

// // UART3: RX, TX
// #define UART3_RX_GPIO_Pin           GPIO_PIN_11
// #define UART3_TX_GPIO_Pin           GPIO_PIN_10
// #define UART3_GPIO_Port             GPIOB
// #define UART3_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE ()
// // #define UART3_GPIO_AF            GPIO_AF7_USART3

// /*
//  *
//  * I2C GPIO Configuration
//  *
//  */

// // Current Sensor I2C: SCL, SDA
// #define I2C1_SCL_GPIO_Pin           GPIO_PIN_6
// #define I2C1_SDA_GPIO_Pin           GPIO_PIN_7
// #define I2C1_GPIO_Port              GPIOB
// #define I2C1_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE ()

#define GPIO_WRITE_PIN(pPORT, PIN, STATE)                                         \
    do {                                                                          \
        (pPORT)->BSRR = ((uint32_t)(PIN) << (16U * ((STATE) == GPIO_PIN_RESET))); \
    } while (0)

typedef struct {
    eBUS_ID_t busId;
    eGPIO_ID_t sckId;
    eGPIO_ID_t misoId;
    eGPIO_ID_t mosiId;
    eGPIO_ID_t nssId;
    uint16_t alternate;
} GPIOSPIInitConf_t;

typedef struct {
    eBUS_ID_t busId;
    eGPIO_ID_t sclId;
    eGPIO_ID_t sdaId;
    uint16_t alternate;
} GPIOI2CInitConf_t;

typedef struct {
    eBUS_ID_t busId;
    eGPIO_ID_t txId;
    eGPIO_ID_t rxId;
    uint16_t alternate;
} GPIOUARTInitConf_t;

typedef struct {
    eTIMER_ID_t timerId;
} GPIOTimerInitConf_t;

typedef struct {
    eGPIO_ID_t gpioId;
    uint32_t ownerId;
    uint16_t mode;
    uint16_t pull;
    uint16_t speed;
    uint32_t alternate;
} GPIOIOInitConf_t;

typedef struct {
    GPIO_TypeDef* pPort;
    uint16_t pin;
    eDEVICE_ID_t ownerId;
} IO_t;

// typedef IO_t volatile vIO_t;
typedef IO_t vIO_t;

vIO_t* GPIOGetIOfromId (eGPIO_ID_t gpioId);
eSTATUS_t GPIOFreeById (eGPIO_ID_t gpioId);
eSTATUS_t GPIOFreeByIO (vIO_t* pIO);
eSTATUS_t GPIOInit (eDEVICE_ID_t ownerId, GPIOBoardConf_t boardConf);
eSTATUS_t GPIOInitSPI (GPIOSPIInitConf_t conf);
eSTATUS_t GPIOInitI2C (GPIOI2CInitConf_t conf);
eSTATUS_t GPIOInitUART (GPIOUARTInitConf_t conf);
eSTATUS_t GPIOInitIO (GPIOIOInitConf_t conf);

#define GPIO_INIT(pSTATUS, OWNER_ID, GPIO_BOARD_CONF)

#define GPIO_INIT_SPI(pSTATUS, BUS_ID, SCK_ID, MISO_ID, MOSI_ID, NSS_ID, ALTERNATE) \
    do {                                                                            \
        GPIOSPIInitConf_t spi = { 0 };                                              \
        spi.busId             = (BUS_ID);                                           \
        spi.sckId             = (SCK_ID);                                           \
        spi.misoId            = (MISO_ID);                                          \
        spi.mosiId            = (MOSI_ID);                                          \
        spi.nssId             = (NSS_ID);                                           \
        spi.alternate         = (ALTERNATE);                                        \
        *(pSTATUS)            = GPIOInitSPI (spi);                                  \
    } while (0)

#define GPIO_INIT_SPI_DATA_ONLY(pSTATUS, BUS_ID, SCK_ID, MISO_ID, MOSI_ID, ALTERNATE) \
    do {                                                                              \
        GPIOSPIInitConf_t spi = { 0 };                                                \
        spi.busId             = (BUS_ID);                                             \
        spi.sckId             = (SCK_ID);                                             \
        spi.misoId            = (MISO_ID);                                            \
        spi.mosiId            = (MOSI_ID);                                            \
        spi.nssId             = eGPIO_ID_NULL;                                        \
        spi.alternate         = (ALTERNATE);                                          \
        *(pSTATUS)            = GPIOInitSPI (spi);                                    \
    } while (0)


#define GPIO_INIT_I2C(pSTATUS, BUS_ID, SCL_ID, SDA_ID, ALTERNATE) \
    do {                                                          \
        GPIOI2CInitConf_t i2c = { 0 };                            \
        i2c.busId             = (BUS_ID);                         \
        i2c.sclId             = (SCL_ID);                         \
        i2c.sdaId             = (SDA_ID);                         \
        i2c.alternate         = (ALTERNATE);                      \
        *(pSTATUS)            = GPIOInitI2C (i2c);                \
    } while (0)

#define GPIO_INIT_UART(pSTATUS, BUS_ID, TX_ID, RX_ID, ALTERNATE) \
    do {                                                         \
        GPIOUARTInitConf_t uart = { 0 };                         \
        uart.busId              = (BUS_ID);                      \
        uart.txId               = (TX_ID);                       \
        uart.rxId               = (RX_ID);                       \
        uart.alternate          = (ALTERNATE);                   \
        *(pSTATUS)              = GPIOInitUART (uart);           \
    } while (0)

#define GPIO_INIT_TIMER(pSTATUS, TIMER_ID, TIMER_GPIO_ID, ALTERNATE) \
    do {                                                             \
        GPIOIOInitConf_t timer = { 0 };                              \
        timer.ownerId          = (TIMER_ID);                         \
        timer.gpioId           = (TIMER_GPIO_ID);                    \
        timer.mode             = GPIO_MODE_AF_PP;                    \
        timer.pull             = GPIO_NOPULL;                        \
        timer.speed            = GPIO_SPEED_FREQ_HIGH;               \
        timer.alternate        = (ALTERNATE);                        \
        *(pSTATUS)             = GPIOInitIO (timer);                 \
    } while (0)

#define GPIO_INIT_EXTI(pSTATUS, EXTI_ID, EXTI_GPIO_ID, ALTERNATE) \
    do {                                                          \
        GPIOIOInitConf_t exti = { 0 };                            \
        exti.ownerId          = (EXTI_ID);                        \
        exti.gpioId           = (EXTI_GPIO_ID);                   \
        exti.mode             = GPIO_MODE_IT_RISING;              \
        exti.pull             = GPIO_NOPULL;                      \
        exti.speed            = GPIO_SPEED_FREQ_HIGH;             \
        exti.alternate        = (ALTERNATE);                      \
        *(pSTATUS)            = GPIOInitIO (exti);                \
    } while (0)

#define GPIO_INIT_OUTPUT(pSTATUS, OWNER_ID, GPIO_ID)    \
    do {                                                \
        GPIOIOInitConf_t output = { 0 };                \
        output.ownerId          = (OWNER_ID);           \
        output.gpioId           = (GPIO_ID);            \
        output.mode             = GPIO_MODE_OUTPUT_PP;  \
        output.pull             = GPIO_PULLUP;          \
        output.speed            = GPIO_SPEED_FREQ_HIGH; \
        output.alternate        = 0U;                   \
        *(pSTATUS)              = GPIOInitIO (output);  \
    } while (0)

#define GPIO_INIT_IO(pSTATUS, OWNER_ID, GPIO_ID, MODE, PULL, SPEED, ALTERNATE) \
    do {                                                                       \
        GPIOIOInitConf_t output = { 0 };                                       \
        output.ownerId          = (OWNER_ID);                                  \
        output.gpioId           = (GPIO_ID);                                   \
        output.mode             = (MODE);                                      \
        output.pull             = (PULL);                                      \
        output.speed            = (SPEED);                                     \
        output.alternate        = (ALTERNATE);                                 \
        *(pSTATUS)              = GPIOInitIO (output);                         \
    } while (0)


#define GPIO_INIT_SPI_SOFTWARE_NSS_ONLY(pSTATUS, OWNER_ID, NSS_ID) \
    GPIO_INIT_OUTPUT (pSTATUS, OWNER_ID, NSS_ID)

#endif // PERIPHS_GPIO_H