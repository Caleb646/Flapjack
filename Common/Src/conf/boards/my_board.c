#include "conf/boards/my_board.h"
#include "conf/board.h"
#include "conf/ids.h"
#include "hal.h"
#include <stdbool.h>


static GPIOBoardConf_t gpioTimer5ch1 = { .id = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_0),
                                         .purpose = eGPIO_PURPOSE_TIMER_CH,
                                         .alternate = GPIO_AF2_TIM5 };
static GPIOBoardConf_t gpioTimer5ch2 = { .id = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_1),
                                         .purpose = eGPIO_PURPOSE_TIMER_CH,
                                         .alternate = GPIO_AF2_TIM5 };
static GPIOBoardConf_t gpioTimer5ch3 = { .id = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_2),
                                         .purpose = eGPIO_PURPOSE_TIMER_CH,
                                         .alternate = GPIO_AF2_TIM5 };
static GPIOBoardConf_t gpioTimer5ch4 = { .id = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_3),
                                         .purpose = eGPIO_PURPOSE_TIMER_CH,
                                         .alternate = GPIO_AF2_TIM5 };
static GPIOBoardConf_t gpioTimer8ch1 = { .id = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_6),
                                         .purpose = eGPIO_PURPOSE_TIMER_CH,
                                         .alternate = GPIO_AF3_TIM8 };
static GPIOBoardConf_t gpioTimer8ch2 = { .id = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_7),
                                         .purpose = eGPIO_PURPOSE_TIMER_CH,
                                         .alternate = GPIO_AF3_TIM8 };
static GPIOBoardConf_t gpioTimer8ch3 = { .id = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_8),
                                         .purpose = eGPIO_PURPOSE_TIMER_CH,
                                         .alternate = GPIO_AF3_TIM8 };
static GPIOBoardConf_t gpioTimer8ch4 = { .id = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_9),
                                         .purpose = eGPIO_PURPOSE_TIMER_CH,
                                         GPIO_AF3_TIM8 };
static GPIOBoardConf_t gpioTimer12ch1 = { .id = GPIO_ID_MAKE (eGPIO_PORTID_H, eGPIO_PINID_6),
                                          .purpose = eGPIO_PURPOSE_TIMER_CH,
                                          .alternate = GPIO_AF2_TIM12 };
static GPIOBoardConf_t gpioTimer12ch2 = { .id = GPIO_ID_MAKE (eGPIO_PORTID_H, eGPIO_PINID_9),
                                          .purpose = eGPIO_PURPOSE_TIMER_CH,
                                          .alternate = GPIO_AF2_TIM12 };

static TimerBoardConf_t timer5ch1 = { TIMER_ID_MAKE (eTIMER_5_DEV_ID, eTIMER_CHANNEL_1_ID),
                                      &gpioTimer5ch1 };
static TimerBoardConf_t timer5ch2 = { TIMER_ID_MAKE (eTIMER_5_DEV_ID, eTIMER_CHANNEL_2_ID),
                                      &gpioTimer5ch2 };
static TimerBoardConf_t timer5ch3 = { TIMER_ID_MAKE (eTIMER_5_DEV_ID, eTIMER_CHANNEL_3_ID),
                                      &gpioTimer5ch3 };
static TimerBoardConf_t timer5ch4 = { TIMER_ID_MAKE (eTIMER_5_DEV_ID, eTIMER_CHANNEL_4_ID),
                                      &gpioTimer5ch4 };
static TimerBoardConf_t timer8ch1 = { TIMER_ID_MAKE (eTIMER_8_DEV_ID, eTIMER_CHANNEL_1_ID),
                                      &gpioTimer8ch1 };
static TimerBoardConf_t timer8ch2 = { TIMER_ID_MAKE (eTIMER_8_DEV_ID, eTIMER_CHANNEL_2_ID),
                                      &gpioTimer8ch2 };
static TimerBoardConf_t timer8ch3 = { TIMER_ID_MAKE (eTIMER_8_DEV_ID, eTIMER_CHANNEL_3_ID),
                                      &gpioTimer8ch3 };
static TimerBoardConf_t timer8ch4 = { TIMER_ID_MAKE (eTIMER_8_DEV_ID, eTIMER_CHANNEL_4_ID),
                                      &gpioTimer8ch4 };
static TimerBoardConf_t timer12ch1 = { TIMER_ID_MAKE (eTIMER_12_DEV_ID, eTIMER_CHANNEL_1_ID),
                                       &gpioTimer12ch1 };
static TimerBoardConf_t timer12ch2 = { TIMER_ID_MAKE (eTIMER_12_DEV_ID, eTIMER_CHANNEL_2_ID),
                                       &gpioTimer12ch2 };


static GPIOBoardConf_t gpioSp1Sck = { .id = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_5),
                                      .purpose   = eGPIO_PURPOSE_SPI_SCK,
                                      .alternate = GPIO_AF5_SPI1 };
static GPIOBoardConf_t gpioSp1Miso = { .id = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_6),
                                       .purpose   = eGPIO_PURPOSE_SPI_MISO,
                                       .alternate = GPIO_AF5_SPI1 };
static GPIOBoardConf_t gpioSp1Mosi = { .id = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_7),
                                       .purpose   = eGPIO_PURPOSE_SPI_MOSI,
                                       .alternate = GPIO_AF5_SPI1 };
static GPIOBoardConf_t gpioSp1Nss_imu = { .id = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_4),
                                          .purpose = eGPIO_PURPOSE_SPI_NSS,
                                          .alternate = 0 };

static GPIOBoardConf_t gpioSp3Sck = { .id = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_10),
                                      .purpose   = eGPIO_PURPOSE_SPI_SCK,
                                      .alternate = GPIO_AF5_SPI3 };
static GPIOBoardConf_t gpioSp3Miso = { .id = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_11),
                                       .purpose   = eGPIO_PURPOSE_SPI_MISO,
                                       .alternate = GPIO_AF5_SPI3 };
static GPIOBoardConf_t gpioSp3Mosi = { .id = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_12),
                                       .purpose   = eGPIO_PURPOSE_SPI_MOSI,
                                       .alternate = GPIO_AF5_SPI3 };
static GPIOBoardConf_t gpioSp3Nss_flash = { .id = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_15),
                                            .purpose = eGPIO_PURPOSE_SPI_NSS,
                                            .alternate = 0 };

static GPIOBoardConf_t gpioSp5Sck = { .id = GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_7),
                                      .purpose   = eGPIO_PURPOSE_SPI_SCK,
                                      .alternate = GPIO_AF5_SPI5 };
static GPIOBoardConf_t gpioSp5Miso = { .id = GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_8),
                                       .purpose   = eGPIO_PURPOSE_SPI_MISO,
                                       .alternate = GPIO_AF5_SPI5 };
static GPIOBoardConf_t gpioSp5Mosi = { .id = GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_9),
                                       .purpose   = eGPIO_PURPOSE_SPI_MOSI,
                                       .alternate = GPIO_AF5_SPI5 };
static GPIOBoardConf_t gpioSp5Nss_baro = { .id = GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_6),
                                           .purpose = eGPIO_PURPOSE_SPI_NSS,
                                           .alternate = 0 };
static GPIOBoardConf_t gpioSp5Nss_mag = { .id = GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_4),
                                          .purpose = eGPIO_PURPOSE_SPI_NSS,
                                          .alternate = 0 };

static SPIDeviceMapping_t spi1Devices[] = {
    { eIMU_DEVICE_ID, &gpioSp1Nss_imu },
};

static SPIDeviceMapping_t spi3Devices[] = {
    { eFLASH_DEVICE_ID, &gpioSp3Nss_flash },
};

static SPIDeviceMapping_t spi5Devices[] = {
    { eBARO_DEVICE_ID, &gpioSp5Nss_baro },
    { eMAG_DEVICE_ID, &gpioSp5Nss_mag },
};

static SPIBoardConf_t spi1 = { { eSPI_1_BUS_ID },
                               .pSckBoardConf     = &gpioSp1Sck,
                               .pMisoBoardConf    = &gpioSp1Miso,
                               .pMosiBoardConf    = &gpioSp1Mosi,
                               .speedKHz          = 1000U, // 1 MHz
                               .pConnectedDevices = spi1Devices,
                               .numConnectedDevices =
                               sizeof (spi1Devices) / sizeof (spi1Devices[0]) };

static SPIBoardConf_t spi3 = { { eSPI_3_BUS_ID },
                               .pSckBoardConf     = &gpioSp3Sck,
                               .pMisoBoardConf    = &gpioSp3Miso,
                               .pMosiBoardConf    = &gpioSp3Mosi,
                               .speedKHz          = 1000U, // 1 MHz
                               .pConnectedDevices = spi3Devices,
                               .numConnectedDevices =
                               sizeof (spi3Devices) / sizeof (spi3Devices[0]) };

static SPIBoardConf_t spi5 = { { eSPI_5_BUS_ID },
                               .pSckBoardConf     = &gpioSp5Sck,
                               .pMisoBoardConf    = &gpioSp5Miso,
                               .pMosiBoardConf    = &gpioSp5Mosi,
                               .speedKHz          = 1000U, // 1 MHz
                               .pConnectedDevices = spi5Devices,
                               .numConnectedDevices =
                               sizeof (spi5Devices) / sizeof (spi5Devices[0]) };


static GPIOBoardConf_t gpioUart1Tx = { GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_14),
                                       GPIO_AF7_USART1 };
static GPIOBoardConf_t gpioUart1Rx = { GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_15),
                                       GPIO_AF7_USART1 };
static GPIOBoardConf_t gpioUart2Tx = { GPIO_ID_MAKE (eGPIO_PORTID_D, eGPIO_PINID_5),
                                       GPIO_AF7_USART2 };
static GPIOBoardConf_t gpioUart2Rx = { GPIO_ID_MAKE (eGPIO_PORTID_D, eGPIO_PINID_6),
                                       GPIO_AF7_USART2 };
static GPIOBoardConf_t gpioUart3Tx = { GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_10),
                                       GPIO_AF7_USART3 };
static GPIOBoardConf_t gpioUart3Rx = { GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_11),
                                       GPIO_AF7_USART3 };

static UARTBoardConf_t uart1 = {
    { eUART_1_BUS_ID },
    .pTxBoardConf = &gpioUart1Tx,
    .pRxBoardConf = &gpioUart1Rx,
    .baudRate     = 115200U,
};
static UARTBoardConf_t uart2 = {
    { eUART_2_BUS_ID },
    .pTxBoardConf = &gpioUart2Tx,
    .pRxBoardConf = &gpioUart2Rx,
    .baudRate     = 115200U,
};
static UARTBoardConf_t uart3 = {
    { eUART_3_BUS_ID },
    .pTxBoardConf = &gpioUart3Tx,
    .pRxBoardConf = &gpioUart3Rx,
    .baudRate     = 115200U,
};


static GPIOBoardConf_t gpioI2C1Scl = { GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_6),
                                       GPIO_AF4_I2C1 };
static GPIOBoardConf_t gpioI2C1Sda = { GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_7),
                                       GPIO_AF4_I2C1 };

static I2CBoardConf_t i2c1 = {
    { eI2C_1_BUS_ID },
    .pSclBoardConf = &gpioI2C1Scl,
    .pSdaBoardConf = &gpioI2C1Sda,
};


static EXTIBoardConf_t exti3_mag = { eEXTI_3_ID, GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_3) };
static EXTIBoardConf_t exti4_baro = { eEXTI_4_ID, GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_10) };
static EXTIBoardConf_t exti5_imu = { eEXTI_5_ID, GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_5) };

static DeviceBoardConf_t imu = { .deviceId = eIMU_DEVICE_ID,
                                 .useDMA   = false,
                                 .generic = { &exti5_imu, (BusHeaderBoardConf_t*)&spi1 } };

static DeviceBoardConf_t mag = { .deviceId = eMAG_DEVICE_ID,
                                 .useDMA   = false,
                                 .generic = { &exti3_mag, (BusHeaderBoardConf_t*)&spi5 } };

static DeviceBoardConf_t baro = { .deviceId = eBARO_DEVICE_ID,
                                  .useDMA   = false,
                                  .generic  = { &exti4_baro,
                                                (BusHeaderBoardConf_t*)&spi5 } };

static DeviceBoardConf_t flash = { .deviceId = eFLASH_DEVICE_ID,
                                   .useDMA   = true,
                                   .generic = { NULL, (BusHeaderBoardConf_t*)&spi3 } };

static DeviceBoardConf_t serialDebug = { .deviceId = eSERIAL_DEBUG_DEVICE_ID,
                                         .useDMA = false,
                                         .generic = { NULL, (BusHeaderBoardConf_t*)&uart1 } };

static DeviceBoardConf_t gps = { .deviceId = eGPS_DEVICE_ID,
                                 .useDMA   = false,
                                 .generic = { NULL, (BusHeaderBoardConf_t*)&uart2 } };

static DeviceBoardConf_t rfReceiver = { .deviceId = eRF_RECEIVER_DEVICE_ID,
                                        .useDMA   = false,
                                        .generic = { NULL, (BusHeaderBoardConf_t*)&uart3 } };

static DeviceBoardConf_t currentSensor = {
    .deviceId = eCURRENT_SENSOR_DEVICE_ID,
    .useDMA   = false,
    .generic  = { NULL, (BusHeaderBoardConf_t*)&i2c1 }
};

static DeviceBoardConf_t leftMotorServo = { .deviceId = eSERVO_1_ID,
                                            .useDMA   = false,
                                            .servo = { .pTimerBoardConf = &timer5ch1,
                                                       .pLinkedMotorBoardConf = NULL,
                                                       .pwmFrequency = 50U,
                                                       .pidRollMix  = 1.0F,
                                                       .pidPitchMix = 1.0F,
                                                       .pidYawMix = 1.0F } };

static DeviceBoardConf_t leftAileronServo = { .deviceId = eSERVO_2_ID,
                                              .useDMA   = false,
                                              .servo = { .pTimerBoardConf = &timer5ch2,
                                                         .pLinkedMotorBoardConf = NULL,
                                                         .pwmFrequency = 50U,
                                                         .pidRollMix = 1.0F,
                                                         .pidPitchMix = 1.0F,
                                                         .pidYawMix = 1.0F } };

static DeviceBoardConf_t elevatorServo = { .deviceId = eSERVO_3_ID,
                                           .useDMA   = false,
                                           .servo = { .pTimerBoardConf = &timer5ch3,
                                                      .pLinkedMotorBoardConf = NULL,
                                                      .pwmFrequency = 50U,
                                                      .pidRollMix   = 1.0F,
                                                      .pidPitchMix  = 1.0F,
                                                      .pidYawMix = 1.0F } };

static DeviceBoardConf_t rudderServo = { .deviceId = eSERVO_4_ID,
                                         .useDMA   = false,
                                         .servo = { .pTimerBoardConf = &timer5ch4,
                                                    .pLinkedMotorBoardConf = NULL,
                                                    .pwmFrequency = 50U,
                                                    .pidRollMix   = 1.0F,
                                                    .pidPitchMix  = 1.0F,
                                                    .pidYawMix = 1.0F } };

static DeviceBoardConf_t rightMotorServo = { .deviceId = eSERVO_5_ID,
                                             .useDMA   = false,
                                             .servo = { .pTimerBoardConf = &timer8ch1,
                                                        .pLinkedMotorBoardConf = NULL,
                                                        .pwmFrequency = 50U,
                                                        .pidRollMix = 1.0F,
                                                        .pidPitchMix = 1.0F,
                                                        .pidYawMix = 1.0F } };

static DeviceBoardConf_t rightAileronServo = { .deviceId = eSERVO_6_ID,
                                               .useDMA   = false,
                                               .servo = { .pTimerBoardConf = &timer8ch2,
                                                          .pLinkedMotorBoardConf = NULL,
                                                          .pwmFrequency = 50U,
                                                          .pidRollMix = 1.0F,
                                                          .pidPitchMix = 1.0F,
                                                          .pidYawMix = 1.0F } };

static DeviceBoardConf_t flightModeServo = { .deviceId = eSERVO_7_ID,
                                             .useDMA   = false,
                                             .servo = { .pTimerBoardConf = &timer8ch3,
                                                        .pLinkedMotorBoardConf = NULL,
                                                        .pwmFrequency = 50U,
                                                        .pidRollMix = 1.0F,
                                                        .pidPitchMix = 1.0F,
                                                        .pidYawMix = 1.0F } };


static DeviceBoardConf_t leftMotor = { .deviceId = eMOTOR_1_ID,
                                       .useDMA   = true,
                                       .motor = { .pTimerBoardConf = &timer12ch1,
                                                  .pLinkedServoBoardConf =
                                                  &leftMotorServo.servo,
                                                  .dshotSpeed  = 150,
                                                  .pidRollMix  = 1.0F,
                                                  .pidPitchMix = 1.0F,
                                                  .pidYawMix   = 1.0F } };

static DeviceBoardConf_t rightMotor = { .deviceId = eMOTOR_2_ID,
                                        .useDMA   = true,
                                        .motor = { .pTimerBoardConf = &timer12ch2,
                                                   .pLinkedServoBoardConf =
                                                   &rightMotorServo.servo,
                                                   .dshotSpeed  = 150,
                                                   .pidRollMix  = 1.0F,
                                                   .pidPitchMix = 1.0F,
                                                   .pidYawMix   = 1.0F } };

static DeviceBoardConf_t* gDeviceBoardConfs[] = { &imu,
                                                  &mag,
                                                  &baro,
                                                  &flash,
                                                  &serialDebug,
                                                  &gps,
                                                  &rfReceiver,
                                                  &currentSensor,
                                                  &leftMotorServo,
                                                  &leftAileronServo,
                                                  &elevatorServo,
                                                  &rudderServo,
                                                  &rightMotorServo,
                                                  &rightAileronServo,
                                                  &flightModeServo,
                                                  &leftMotor,
                                                  &rightMotor };

static bool isInitialized     = false;
static BoardConf_t gBoardConf = { 0 };

bool BoardConfInit (void) {

    // link left and right motors to their servos
    leftMotor.motor.pLinkedServoBoardConf  = &leftMotorServo;
    rightMotor.motor.pLinkedServoBoardConf = &rightMotorServo;
    // link left and right servos to their motors
    leftMotorServo.servo.pLinkedMotorBoardConf  = &leftMotor;
    rightMotorServo.servo.pLinkedMotorBoardConf = &rightMotor;

    gBoardConf.ppDeviceBoardConfs = gDeviceBoardConfs;
    gBoardConf.numDevices =
    sizeof (gDeviceBoardConfs) / sizeof (gDeviceBoardConfs[0]);

    isInitialized = true;
    return true;
}

// clang-format on

BoardConf_t* BoardConfGet (void) {

    if (isInitialized == false) {
        return NULL;
    }
    return &gBoardConf;
}

DeviceBoardConf_t* BoardConfGetDeviceById (eDEVICE_ID_t deviceId) {

    for (uint32_t i = 0; i < gBoardConf.numDevices; ++i) {
        if (gBoardConf.ppDeviceBoardConfs[i]->deviceId == deviceId) {
            return gBoardConf.ppDeviceBoardConfs[i];
        }
    }
    return 0;
}