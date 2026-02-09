// #include "conf/board.h"
// #include "conf/ids.h"
// #include "hal.h"
// #include <stdbool.h>
// #include <stdint.h>

// /*
//  * Timer Configuration
//  */

// static GPIOSharedConf_t gpioShared_Tim5  = { .mode      = GPIO_MODE_AF_PP,
//                                              .pull      = GPIO_NOPULL,
//                                              .speed     = GPIO_SPEED_FREQ_LOW,
//                                              .alternate = GPIO_AF2_TIM5 };
// static GPIOSharedConf_t gpioShared_Tim8  = { .mode      = GPIO_MODE_AF_PP,
//                                              .pull      = GPIO_NOPULL,
//                                              .speed     = GPIO_SPEED_FREQ_LOW,
//                                              .alternate = GPIO_AF3_TIM8 };
// static GPIOSharedConf_t gpioShared_Tim12 = {
//     .mode      = GPIO_MODE_AF_PP, // GPIO_MODE_OUTPUT_PP using Bitbanging not DMA
//     .pull      = GPIO_NOPULL,
//     .speed     = GPIO_SPEED_FREQ_LOW,
//     .alternate = GPIO_AF2_TIM12 // 0U
// };

// static GPIOBoardConf_t gpioTimer5ch1 = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_0),
//                                          .pShared = &gpioShared_Tim5 };
// static GPIOBoardConf_t gpioTimer5ch2 = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_1),
//                                          .pShared = &gpioShared_Tim5 };
// static GPIOBoardConf_t gpioTimer5ch3 = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_2),
//                                          .pShared = &gpioShared_Tim5 };
// static GPIOBoardConf_t gpioTimer5ch4 = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_3),
//                                          .pShared = &gpioShared_Tim5 };

// static GPIOBoardConf_t gpioTimer8ch1 = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_6),
//                                          .pShared = &gpioShared_Tim8 };
// static GPIOBoardConf_t gpioTimer8ch2 = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_7),
//                                          .pShared = &gpioShared_Tim8 };
// static GPIOBoardConf_t gpioTimer8ch3 = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_8),
//                                          .pShared = &gpioShared_Tim8 };
// // static GPIOBoardConf_t gpioTimer8ch4 = { .id = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_9),
// //                                          .pShared = &gpioShared_Tim8 };

// static GPIOBoardConf_t gpioTimer12ch1 = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_H, eGPIO_PINID_6),
//                                           .pShared = &gpioShared_Tim12 };
// static GPIOBoardConf_t gpioTimer12ch2 = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_H, eGPIO_PINID_9),
//                                           .pShared = &gpioShared_Tim12 };

// static TimerBoardConf_t timer5ch1 = { TIMER_ID_MAKE (eTIMER_5_DEVICE_ID, eTIMER_CHANNEL_1_ID),
// &gpioTimer5ch1 }; static TimerBoardConf_t timer5ch2 = { TIMER_ID_MAKE (eTIMER_5_DEVICE_ID,
// eTIMER_CHANNEL_2_ID), &gpioTimer5ch2 }; static TimerBoardConf_t timer5ch3 = { TIMER_ID_MAKE
// (eTIMER_5_DEVICE_ID, eTIMER_CHANNEL_3_ID), &gpioTimer5ch3 }; static TimerBoardConf_t timer5ch4 =
// { TIMER_ID_MAKE (eTIMER_5_DEVICE_ID, eTIMER_CHANNEL_4_ID), &gpioTimer5ch4 }; static
// TimerBoardConf_t timer8ch1 = { TIMER_ID_MAKE (eTIMER_8_DEVICE_ID, eTIMER_CHANNEL_1_ID),
// &gpioTimer8ch1 }; static TimerBoardConf_t timer8ch2 = { TIMER_ID_MAKE (eTIMER_8_DEVICE_ID,
// eTIMER_CHANNEL_2_ID), &gpioTimer8ch2 }; static TimerBoardConf_t timer8ch3 = { TIMER_ID_MAKE
// (eTIMER_8_DEVICE_ID, eTIMER_CHANNEL_3_ID), &gpioTimer8ch3 };
// // static TimerBoardConf_t timer8ch4 = { TIMER_ID_MAKE (eTIMER_8_DEV_ID, eTIMER_CHANNEL_4_ID),
// //                                       &gpioTimer8ch4 };
// static TimerBoardConf_t timer12ch1 = { TIMER_ID_MAKE (eTIMER_12_DEVICE_ID, eTIMER_CHANNEL_1_ID),
// &gpioTimer12ch1 }; static TimerBoardConf_t timer12ch2 = { TIMER_ID_MAKE (eTIMER_12_DEVICE_ID,
// eTIMER_CHANNEL_2_ID), &gpioTimer12ch2 };


// /*
//  * SPI Configuration
//  */
// static GPIOSharedConf_t gpioShared_Spi_1   = { .mode      = GPIO_MODE_AF_PP,
//                                                .pull      = GPIO_NOPULL,
//                                                .speed     = GPIO_SPEED_FREQ_VERY_HIGH,
//                                                .alternate = GPIO_AF5_SPI1 };
// static GPIOSharedConf_t gpioShared_Spi_3   = { .mode      = GPIO_MODE_AF_PP,
//                                                .pull      = GPIO_NOPULL,
//                                                .speed     = GPIO_SPEED_FREQ_VERY_HIGH,
//                                                .alternate = GPIO_AF5_SPI3 };
// static GPIOSharedConf_t gpioShared_Spi_5   = { .mode      = GPIO_MODE_AF_PP,
//                                                .pull      = GPIO_NOPULL,
//                                                .speed     = GPIO_SPEED_FREQ_VERY_HIGH,
//                                                .alternate = GPIO_AF5_SPI5 };
// static GPIOSharedConf_t gpioShared_Spi_Nss = { .mode      = GPIO_MODE_OUTPUT_PP,
//                                                .pull      = GPIO_NOPULL,
//                                                .speed     = GPIO_SPEED_FREQ_VERY_HIGH,
//                                                .alternate = 0 };

// static GPIOBoardConf_t gpioSp1Sck     = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_5),
//                                           .pShared = &gpioShared_Spi_1 };
// static GPIOBoardConf_t gpioSp1Miso    = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_6),
//                                           .pShared = &gpioShared_Spi_1 };
// static GPIOBoardConf_t gpioSp1Mosi    = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_7),
//                                           .pShared = &gpioShared_Spi_1 };
// static GPIOBoardConf_t gpioSp1Nss_imu = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_4),
//                                           .pShared = &gpioShared_Spi_Nss };

// static GPIOBoardConf_t gpioSp3Sck       = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_10),
//                                             .pShared = &gpioShared_Spi_3 };
// static GPIOBoardConf_t gpioSp3Miso      = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_11),
//                                             .pShared = &gpioShared_Spi_3 };
// static GPIOBoardConf_t gpioSp3Mosi      = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_12),
//                                             .pShared = &gpioShared_Spi_3 };
// static GPIOBoardConf_t gpioSp3Nss_flash = { .id = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_15),
//                                             .pShared = &gpioShared_Spi_Nss };

// static GPIOBoardConf_t gpioSp5Sck      = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_7),
//                                            .pShared = &gpioShared_Spi_5 };
// static GPIOBoardConf_t gpioSp5Miso     = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_8),
//                                            .pShared = &gpioShared_Spi_5 };
// static GPIOBoardConf_t gpioSp5Mosi     = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_9),
//                                            .pShared = &gpioShared_Spi_5 };
// static GPIOBoardConf_t gpioSp5Nss_baro = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_6),
//                                            .pShared = &gpioShared_Spi_Nss };
// static GPIOBoardConf_t gpioSp5Nss_mag  = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_4),
//                                            .pShared = &gpioShared_Spi_Nss };

// static SPIDeviceMapping_t spi1Devices[] = {
//     { eIMU_DEVICE_ID, &gpioSp1Nss_imu },
// };

// static SPIDeviceMapping_t spi3Devices[] = {
//     { eFLASH_DEVICE_ID, &gpioSp3Nss_flash },
// };

// static SPIDeviceMapping_t spi5Devices[] = {
//     { eBARO_DEVICE_ID, &gpioSp5Nss_baro },
//     { eMAG_DEVICE_ID, &gpioSp5Nss_mag },
// };

// static BusBoardConf_t spi1 = { .busId        = eSPI_1_BUS_ID,
//                                .SPIBoardConf = { .pSckBoardConf     = &gpioSp1Sck,
//                                                  .pMisoBoardConf    = &gpioSp1Miso,
//                                                  .pMosiBoardConf    = &gpioSp1Mosi,
//                                                  .speedKHz          = 1000U, // 1 MHz
//                                                  .pConnectedDevices = spi1Devices,
//                                                  .numConnectedDevices =
//                                                  sizeof (spi1Devices) / sizeof (spi1Devices[0]) } };

// static BusBoardConf_t spi3 = { .busId        = eSPI_3_BUS_ID,
//                                .SPIBoardConf = { .pSckBoardConf     = &gpioSp3Sck,
//                                                  .pMisoBoardConf    = &gpioSp3Miso,
//                                                  .pMosiBoardConf    = &gpioSp3Mosi,
//                                                  .speedKHz          = 1000U, // 1 MHz
//                                                  .pConnectedDevices = spi3Devices,
//                                                  .numConnectedDevices =
//                                                  sizeof (spi3Devices) / sizeof (spi3Devices[0]) } };

// static BusBoardConf_t spi5 = { .busId        = eSPI_5_BUS_ID,
//                                .SPIBoardConf = { .pSckBoardConf     = &gpioSp5Sck,
//                                                  .pMisoBoardConf    = &gpioSp5Miso,
//                                                  .pMosiBoardConf    = &gpioSp5Mosi,
//                                                  .speedKHz          = 1000U, // 1 MHz
//                                                  .pConnectedDevices = spi5Devices,
//                                                  .numConnectedDevices =
//                                                  sizeof (spi5Devices) / sizeof (spi5Devices[0]) } };
// /*
//  * UART Configuration
//  */
// static GPIOSharedConf_t gpioShared_Uart_1 = { .mode      = GPIO_MODE_AF_PP,
//                                               .pull      = GPIO_NOPULL,
//                                               .speed     = GPIO_SPEED_FREQ_VERY_HIGH,
//                                               .alternate = GPIO_AF7_USART1 };
// static GPIOSharedConf_t gpioShared_Uart_2 = { .mode      = GPIO_MODE_AF_PP,
//                                               .pull      = GPIO_NOPULL,
//                                               .speed     = GPIO_SPEED_FREQ_VERY_HIGH,
//                                               .alternate = GPIO_AF7_USART2 };
// static GPIOSharedConf_t gpioShared_Uart_3 = { .mode      = GPIO_MODE_AF_PP,
//                                               .pull      = GPIO_NOPULL,
//                                               .speed     = GPIO_SPEED_FREQ_VERY_HIGH,
//                                               .alternate = GPIO_AF7_USART3 };

// static GPIOBoardConf_t gpioUart1Tx = { GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_14), .pShared = &gpioShared_Uart_1 };
// static GPIOBoardConf_t gpioUart1Rx = { GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_15), .pShared = &gpioShared_Uart_1 };

// static GPIOBoardConf_t gpioUart2Tx = { GPIO_ID_MAKE (eGPIO_PORTID_D, eGPIO_PINID_5), .pShared = &gpioShared_Uart_2 };
// static GPIOBoardConf_t gpioUart2Rx = { GPIO_ID_MAKE (eGPIO_PORTID_D, eGPIO_PINID_6), .pShared = &gpioShared_Uart_2 };

// static GPIOBoardConf_t gpioUart3Tx = { GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_10), .pShared = &gpioShared_Uart_3 };
// static GPIOBoardConf_t gpioUart3Rx = { GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_11), .pShared = &gpioShared_Uart_3 };

// static BusBoardConf_t uart1 = { .busId         = eUART_1_BUS_ID,
//                                 .UARTBoardConf = {
//                                 .pTxBoardConf = &gpioUart1Tx,
//                                 .pRxBoardConf = &gpioUart1Rx,
//                                 .baudRate     = 115200U,
//                                 } };

// static BusBoardConf_t uart2 = { .busId         = eUART_2_BUS_ID,
//                                 .UARTBoardConf = {
//                                 .pTxBoardConf = &gpioUart2Tx,
//                                 .pRxBoardConf = &gpioUart2Rx,
//                                 .baudRate     = 115200U,
//                                 } };

// static BusBoardConf_t uart3 = { .busId         = eUART_3_BUS_ID,
//                                 .UARTBoardConf = {
//                                 .pTxBoardConf = &gpioUart3Tx,
//                                 .pRxBoardConf = &gpioUart3Rx,
//                                 .baudRate     = 115200U,
//                                 } };


// /*
//  * I2C Configuration
//  */
// static GPIOSharedConf_t gpioShared_I2C_1 = { .mode      = GPIO_MODE_AF_OD,
//                                              .pull      = GPIO_PULLUP,
//                                              .speed     = GPIO_SPEED_FREQ_VERY_HIGH,
//                                              .alternate = GPIO_AF4_I2C1 };
// static GPIOBoardConf_t gpioI2C1Scl = { GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_6), .pShared = &gpioShared_I2C_1 };
// static GPIOBoardConf_t gpioI2C1Sda = { GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_7), .pShared = &gpioShared_I2C_1 };

// static BusBoardConf_t i2c1 = { .busId        = eI2C_1_BUS_ID,
//                                .I2CBoardConf = {
//                                .pSclBoardConf = &gpioI2C1Scl,
//                                .pSdaBoardConf = &gpioI2C1Sda,
//                                } };
// /*
//  * EXTI Configuration
//  */
// static GPIOSharedConf_t gpioShared_Exti = { .mode      = GPIO_MODE_IT_RISING,
//                                             .pull      = GPIO_NOPULL,
//                                             .speed     = GPIO_SPEED_FREQ_LOW,
//                                             .alternate = 0 };
// static GPIOBoardConf_t gpioExti3_mag    = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_3),
//                                             .pShared = &gpioShared_Exti };
// static GPIOBoardConf_t gpioExti4_baro   = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_10),
//                                             .pShared = &gpioShared_Exti };
// static GPIOBoardConf_t gpioExti5_imu    = { .id      = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_5),
//                                             .pShared = &gpioShared_Exti };
// static EXTIBoardConf_t exti3_mag        = { eEXTI_3_ID, .pGPIOBoardConf = &gpioExti3_mag };
// static EXTIBoardConf_t exti4_baro       = { eEXTI_4_ID, .pGPIOBoardConf = &gpioExti4_baro };
// static EXTIBoardConf_t exti5_imu        = { eEXTI_5_ID, .pGPIOBoardConf = &gpioExti5_imu };


// /*
//  * Device Configuration
//  */
// static DeviceBoardConf_t imu = { .deviceId = eIMU_DEVICE_ID, .generic = { &exti5_imu, &spi1 } };

// static DeviceBoardConf_t mag = { .deviceId = eMAG_DEVICE_ID, .generic = { &exti3_mag, &spi5 } };

// static DeviceBoardConf_t baro = { .deviceId = eBARO_DEVICE_ID, .generic = { &exti4_baro, &spi5 } };

// static DeviceBoardConf_t flash = { .deviceId = eFLASH_DEVICE_ID, .generic = { NULL, &spi3 } };

// static DeviceBoardConf_t serialDebug = { .deviceId = eSERIAL_DEBUG_DEVICE_ID, .generic = { NULL, &uart1 } };

// static DeviceBoardConf_t gps = { .deviceId = eGPS_DEVICE_ID, .generic = { NULL, &uart2 } };

// static DeviceBoardConf_t rfReceiver = { .deviceId = eRF_RECEIVER_DEVICE_ID, .generic = { NULL, &uart3 } };

// static DeviceBoardConf_t currentSensor = { .deviceId = eCURRENT_SENSOR_DEVICE_ID, .generic = { NULL, &i2c1 } };

// static DeviceBoardConf_t leftMotorServo = { .deviceId = eSERVO_1_ID,
//                                             .servo    = { .pTimerBoardConf       = &timer5ch1,
//                                                           .pLinkedMotorBoardConf = NULL,
//                                                           .pwmFrequency          = 50U,
//                                                           .pidRollMix            = 1.0F,
//                                                           .pidPitchMix           = 1.0F,
//                                                           .pidYawMix             = 1.0F } };

// static DeviceBoardConf_t leftAileronServo = { .deviceId = eSERVO_2_ID,
//                                               .servo    = { .pTimerBoardConf       = &timer5ch2,
//                                                             .pLinkedMotorBoardConf = NULL,
//                                                             .pwmFrequency          = 50U,
//                                                             .pidRollMix            = 1.0F,
//                                                             .pidPitchMix           = 1.0F,
//                                                             .pidYawMix             = 1.0F } };

// static DeviceBoardConf_t elevatorServo = { .deviceId = eSERVO_3_ID,
//                                            .servo    = { .pTimerBoardConf       = &timer5ch3,
//                                                          .pLinkedMotorBoardConf = NULL,
//                                                          .pwmFrequency          = 50U,
//                                                          .pidRollMix            = 1.0F,
//                                                          .pidPitchMix           = 1.0F,
//                                                          .pidYawMix             = 1.0F } };

// static DeviceBoardConf_t rudderServo = { .deviceId = eSERVO_4_ID,
//                                          .servo    = { .pTimerBoardConf       = &timer5ch4,
//                                                        .pLinkedMotorBoardConf = NULL,
//                                                        .pwmFrequency          = 50U,
//                                                        .pidRollMix            = 1.0F,
//                                                        .pidPitchMix           = 1.0F,
//                                                        .pidYawMix             = 1.0F } };

// static DeviceBoardConf_t rightMotorServo = { .deviceId = eSERVO_5_ID,
//                                              .servo    = { .pTimerBoardConf       = &timer8ch1,
//                                                            .pLinkedMotorBoardConf = NULL,
//                                                            .pwmFrequency          = 50U,
//                                                            .pidRollMix            = 1.0F,
//                                                            .pidPitchMix           = 1.0F,
//                                                            .pidYawMix             = 1.0F } };

// static DeviceBoardConf_t rightAileronServo = { .deviceId = eSERVO_6_ID,
//                                                .servo    = { .pTimerBoardConf       = &timer8ch2,
//                                                              .pLinkedMotorBoardConf = NULL,
//                                                              .pwmFrequency          = 50U,
//                                                              .pidRollMix            = 1.0F,
//                                                              .pidPitchMix           = 1.0F,
//                                                              .pidYawMix             = 1.0F } };

// static DeviceBoardConf_t flightModeServo = { .deviceId = eSERVO_7_ID,
//                                              .servo    = { .pTimerBoardConf       = &timer8ch3,
//                                                            .pLinkedMotorBoardConf = NULL,
//                                                            .pwmFrequency          = 50U,
//                                                            .pidRollMix            = 1.0F,
//                                                            .pidPitchMix           = 1.0F,
//                                                            .pidYawMix             = 1.0F } };

// static DeviceBoardConf_t leftMotor = { .deviceId = eMOTOR_1_ID,
//                                        .motor    = { .pTimerBoardConf       = &timer12ch1,
//                                                      .pLinkedServoBoardConf = NULL,
//                                                      .useDMA                = false,
//                                                      .dshotSpeed            = eDSHOT_TYPE_150,
//                                                      .pidRollMix            = 1.0F,
//                                                      .pidPitchMix           = 1.0F,
//                                                      .pidYawMix             = 1.0F } };

// static DeviceBoardConf_t rightMotor = { .deviceId = eMOTOR_2_ID,
//                                         .motor    = { .pTimerBoardConf       = &timer12ch2,
//                                                       .pLinkedServoBoardConf = NULL,
//                                                       .useDMA                = false,
//                                                       .dshotSpeed            = eDSHOT_TYPE_150,
//                                                       .pidRollMix            = 1.0F,
//                                                       .pidPitchMix           = 1.0F,
//                                                       .pidYawMix             = 1.0F } };

// static DeviceBoardConf_t* gDeviceBoardConfs[] = { &imu,
//                                                   &mag,
//                                                   &baro,
//                                                   &flash,
//                                                   &serialDebug,
//                                                   &gps,
//                                                   &rfReceiver,
//                                                   &currentSensor,
//                                                   &leftMotorServo,
//                                                   &leftAileronServo,
//                                                   &elevatorServo,
//                                                   &rudderServo,
//                                                   &rightMotorServo,
//                                                   &rightAileronServo,
//                                                   &flightModeServo,
//                                                   &leftMotor,
//                                                   &rightMotor };

// bool BoardConfInit_MyBoard (void) {

//     // link left and right motors to their servos
//     leftMotor.motor.pLinkedServoBoardConf  = &leftMotorServo;
//     rightMotor.motor.pLinkedServoBoardConf = &rightMotorServo;
//     // link left and right servos to their motors
//     leftMotorServo.servo.pLinkedMotorBoardConf  = &leftMotor;
//     rightMotorServo.servo.pLinkedMotorBoardConf = &rightMotor;

//     ge_BoardConf.ppDeviceBoardConfs = gDeviceBoardConfs;
//     ge_BoardConf.numDevices         = sizeof (gDeviceBoardConfs) / sizeof (gDeviceBoardConfs[0]);

//     ge_isBoardConfInitialized = true;
//     return true;
// }