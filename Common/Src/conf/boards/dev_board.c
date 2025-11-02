#include "conf/board.h"
#include "conf/ids.h"
#include "hal.h"
#include <stdbool.h>
#include <stdint.h>


// #define TIM8_CH1_GPIO GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_6)

/*
 * Servo Configuration
 */
static GPIOBoardConf_t gpioTimer8ch1 = { .id   = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_6),
                                         .conf = {
                                         .mode      = GPIO_MODE_AF_PP, // GPIO_MODE_OUTPUT_PP
                                         .pull      = GPIO_NOPULL,
                                         .speed     = GPIO_SPEED_FREQ_HIGH,
                                         .alternate = GPIO_AF3_TIM8, // 0U
                                         } };

static TimerBoardConf_t timer8ch1 = { TIMER_ID_MAKE (eTIMER_8_DEVICE_ID, eTIMER_CHANNEL_1_ID), &gpioTimer8ch1 };

static GPIOBoardConf_t gpioTimer13ch1 = { .id   = GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_8),
                                          .conf = {
                                          .mode      = GPIO_MODE_AF_PP,
                                          .pull      = GPIO_NOPULL,
                                          .speed     = GPIO_SPEED_FREQ_HIGH,
                                          .alternate = GPIO_AF9_TIM13,
                                          } };

static TimerBoardConf_t timer13ch1 = { TIMER_ID_MAKE (eTIMER_13_DEVICE_ID, eTIMER_CHANNEL_1_ID), &gpioTimer13ch1 };

static DeviceBoardConf_t leftMotorServo = { .deviceId = eSERVO_1_ID,
                                            .servo = { .pTimerBoardConf = &timer13ch1, // &timer8ch1,
                                                       .pLinkedMotorBoardConf = NULL,
                                                       .pwmFrequency          = 50U,
                                                       .pidRollMix            = 1.0F,
                                                       .pidPitchMix           = 1.0F,
                                                       .pidYawMix             = 1.0F } };
/*
 * Motor Configuration
 */
static DeviceBoardConf_t leftMotor = { .deviceId = eMOTOR_1_ID,
                                       .motor    = { .pTimerBoardConf = &timer8ch1, // &timer13ch1,
                                                     .pLinkedServoBoardConf = NULL,
                                                     .useDMA                = false,
                                                     .dshotSpeed            = 150,
                                                     .pidRollMix            = 1.0F,
                                                     .pidPitchMix           = 1.0F,
                                                     .pidYawMix             = 1.0F } };

// #define TIM13_CH1_GPIO GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_8)

/*
 * IMU Configuration
 */
// #define SPI2_SCK_GPIO  GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_12)
// #define SPI2_MISO_GPIO GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_2)
// #define SPI2_MOSI_GPIO GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_3)
// #define IMU_NSS_GPIO   GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_11)
// #define IMU_INT_EXTI   eEXTI_7_ID
// #define IMU_INT_GPIO   GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_7)
// #define IMU_BUS_ID     eSPI_2_BUS_ID

static GPIOSharedConf_t gpioShared_Spi_2 = { .mode      = GPIO_MODE_AF_PP,
                                             .pull      = GPIO_NOPULL,
                                             .speed     = GPIO_SPEED_FREQ_VERY_HIGH,
                                             .alternate = GPIO_AF5_SPI1 };

static GPIOSharedConf_t gpioShared_Spi_Nss = { .mode      = GPIO_MODE_OUTPUT_PP,
                                               .pull      = GPIO_NOPULL,
                                               .speed     = GPIO_SPEED_FREQ_VERY_HIGH,
                                               .alternate = 0 };

static GPIOBoardConf_t gpioSp2Sck  = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_12),
                                       .pShared = &gpioShared_Spi_2 };
static GPIOBoardConf_t gpioSp2Miso = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_2),
                                       .pShared = &gpioShared_Spi_2 };
static GPIOBoardConf_t gpioSp2Mosi = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_3),
                                       .pShared = &gpioShared_Spi_2 };
static GPIOBoardConf_t gpioSp2Nss  = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_11),
                                       .pShared = &gpioShared_Spi_Nss };

static SPIDeviceMapping_t spi2Devices[] = {
    { eIMU_DEVICE_ID, &gpioSp2Nss },
};

static BusBoardConf_t spi2   = { .busId        = eSPI_2_BUS_ID,
                                 .SPIBoardConf = { .pSckBoardConf     = &gpioSp2Sck,
                                                   .pMisoBoardConf    = &gpioSp2Miso,
                                                   .pMosiBoardConf    = &gpioSp2Mosi,
                                                   .speedKHz          = 1000U, // 1 MHz
                                                   .pConnectedDevices = spi2Devices,
                                                   .numConnectedDevices =
                                                   sizeof (spi2Devices) / sizeof (spi2Devices[0]) } };
static DeviceBoardConf_t imu = { .deviceId = eIMU_DEVICE_ID, .generic = { NULL, &spi2 } };


/*
 * Serial Debug Configuration
 */
#define UART_1_RX_GPIO GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_9)
#define UART_1_TX_GPIO GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_10)

static GPIOSharedConf_t gpioShared_Uart_1 = { .mode      = GPIO_MODE_AF_PP,
                                              .pull      = GPIO_NOPULL,
                                              .speed     = GPIO_SPEED_FREQ_LOW,
                                              .alternate = GPIO_AF7_USART1 };
static GPIOBoardConf_t gpioUart1Tx = { GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_10), .pShared = &gpioShared_Uart_1 };
static GPIOBoardConf_t gpioUart1Rx = { GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_9), .pShared = &gpioShared_Uart_1 };
static BusBoardConf_t uart1 = { .busId         = eUART_1_BUS_ID,
                                .UARTBoardConf = {
                                .pTxBoardConf = &gpioUart1Tx,
                                .pRxBoardConf = &gpioUart1Rx,
                                .baudRate     = 230400U,
                                } };

static DeviceBoardConf_t serialDebug = { .deviceId = eSERIAL_DEBUG_DEVICE_ID, .generic = { NULL, &uart1 } };


static DeviceBoardConf_t* gDeviceBoardConfs[] = { &imu, &serialDebug, &leftMotorServo, &leftMotor };

bool BoardConfInit_DevBoard (void) {

    // link left motor to left servo
    leftMotor.motor.pLinkedServoBoardConf = &leftMotorServo;
    // link left servo to left motor
    leftMotorServo.servo.pLinkedMotorBoardConf = &leftMotor;

    ge_BoardConf.ppDeviceBoardConfs = gDeviceBoardConfs;
    ge_BoardConf.numDevices         = sizeof (gDeviceBoardConfs) / sizeof (gDeviceBoardConfs[0]);

    ge_isBoardConfInitialized = true;
    return true;
}