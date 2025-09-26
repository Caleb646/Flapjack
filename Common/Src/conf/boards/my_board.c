#include "conf/boards/my_board.h"
#include "conf/board.h"
#include "conf/ids.h"
#include "hal.h"

static TimerBoardConf_t gTimerBoardConfs[] = {

    { TIMER_ID_MAKE (eTIMER_5_DEV_ID, eTIMER_CHANNEL_1_ID),
      GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_0),
      GPIO_AF2_TIM5 },
    { TIMER_ID_MAKE (eTIMER_5_DEV_ID, eTIMER_CHANNEL_2_ID),
      GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_1),
      GPIO_AF2_TIM5 },
    { TIMER_ID_MAKE (eTIMER_5_DEV_ID, eTIMER_CHANNEL_3_ID),
      GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_2),
      GPIO_AF2_TIM5 },
    { TIMER_ID_MAKE (eTIMER_5_DEV_ID, eTIMER_CHANNEL_4_ID),
      GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_3),
      GPIO_AF2_TIM5 },

    { TIMER_ID_MAKE (eTIMER_8_DEV_ID, eTIMER_CHANNEL_1_ID),
      GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_6),
      GPIO_AF3_TIM8 },
    { TIMER_ID_MAKE (eTIMER_8_DEV_ID, eTIMER_CHANNEL_2_ID),
      GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_7),
      GPIO_AF3_TIM8 },
    { TIMER_ID_MAKE (eTIMER_8_DEV_ID, eTIMER_CHANNEL_3_ID),
      GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_8),
      GPIO_AF3_TIM8 },
    { TIMER_ID_MAKE (eTIMER_8_DEV_ID, eTIMER_CHANNEL_4_ID),
      GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_9),
      GPIO_AF3_TIM8 },

    { TIMER_ID_MAKE (eTIMER_12_DEV_ID, eTIMER_CHANNEL_1_ID),
      GPIO_ID_MAKE (eGPIO_PORTID_H, eGPIO_PINID_6),
      GPIO_AF2_TIM12 },
    { TIMER_ID_MAKE (eTIMER_12_DEV_ID, eTIMER_CHANNEL_2_ID),
      GPIO_ID_MAKE (eGPIO_PORTID_H, eGPIO_PINID_9),
      GPIO_AF2_TIM12 },
};

#define TIM5_CH1_IDX  0
#define TIM5_CH2_IDX  1
#define TIM5_CH3_IDX  2
#define TIM5_CH4_IDX  3
#define TIM8_CH1_IDX  4
#define TIM8_CH2_IDX  5
#define TIM8_CH3_IDX  6
#define TIM8_CH4_IDX  7
#define TIM12_CH1_IDX 8
#define TIM12_CH2_IDX 9

static SPIBoardConf_t gSPIBoardConfs[] = {

    {
    { eSPI_1_BUS_ID },
    GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_5),
    GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_6),
    GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_7),
    1000U, // 1 MHz
    GPIO_AF5_SPI1,
    },

    {
    { eSPI_3_BUS_ID },
    GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_10),
    GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_11),
    GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_12),
    1000U, // 1 MHz
    GPIO_AF5_SPI3,
    },
    {
    { eSPI_5_BUS_ID },
    GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_7),
    GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_8),
    GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_9),
    1000U, // 1 MHz
    GPIO_AF5_SPI5,
    }
};

#define SPI1_IDX 0
#define SPI3_IDX 1
#define SPI5_IDX 2

static UARTBoardConf_t gUARTBoardConfs[] = {

    { { eUART_1_BUS_ID },
      GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_14),
      GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_15),
      GPIO_AF7_USART1,
      115200U },

    { { eUART_2_BUS_ID },
      GPIO_ID_MAKE (eGPIO_PORTID_D, eGPIO_PINID_5),
      GPIO_ID_MAKE (eGPIO_PORTID_D, eGPIO_PINID_6),
      GPIO_AF7_USART2,
      115200U },

    { { eUART_3_BUS_ID },
      GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_10),
      GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_11),
      GPIO_AF7_USART3,
      115200U }
};

#define UART1_IDX 0
#define UART2_IDX 1
#define UART3_IDX 2

static I2CBoardConf_t gI2CBoardConfs[] = {

    { { eI2C_1_BUS_ID },
      GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_6),
      GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_7),
      GPIO_AF4_I2C1 }
};

#define I2C1_IDX 0

static DeviceBoardConf_t gDeviceBoardConfs[] = {

    { eIMU_DEVICE_ID,
      { eEXTI_5_ID, GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_5) },
      (BusHeaderBoardConf_t*)&gSPIBoardConfs[SPI1_IDX],
      GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_4) },

    { eMAG_DEVICE_ID,
      { eEXTI_3_ID, GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_3) },
      (BusHeaderBoardConf_t*)&gSPIBoardConfs[SPI5_IDX],
      GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_4) },

    { eBARO_DEVICE_ID,
      { eEXTI_4_ID, GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_10) },
      (BusHeaderBoardConf_t*)&gSPIBoardConfs[SPI5_IDX],
      GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_6) },

    { eFLASH_DEVICE_ID,
      { eEXTI_ID_NULL, eGPIO_ID_NULL },
      (BusHeaderBoardConf_t*)&gSPIBoardConfs[SPI3_IDX],
      GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_10) },

    { eSERIAL_DEBUG_DEVICE_ID,
      { eEXTI_ID_NULL, eGPIO_ID_NULL },
      (BusHeaderBoardConf_t*)&gUARTBoardConfs[UART1_IDX],
      GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_1) },

    { eGPS_DEVICE_ID,
      { eEXTI_ID_NULL, eGPIO_ID_NULL },
      (BusHeaderBoardConf_t*)&gUARTBoardConfs[UART2_IDX],
      eGPIO_ID_NULL },

    { eRF_RECEIVER_DEVICE_ID,
      { eEXTI_ID_NULL, eGPIO_ID_NULL },
      (BusHeaderBoardConf_t*)&gUARTBoardConfs[UART3_IDX],
      eGPIO_ID_NULL },

    { eCURRENT_SENSOR_DEVICE_ID,
      { eEXTI_ID_NULL, eGPIO_ID_NULL },
      (BusHeaderBoardConf_t*)&gI2CBoardConfs[I2C1_IDX],
      eGPIO_ID_NULL }
};

static ServoBoardConf_t gServoBoardConfs[] = {
    // Left servo motor
    { eSERVO_1_ID, &gTimerBoardConfs[TIM5_CH1_IDX], 0, 50, 1.0F, 1.0F, 1.0F },
    // Left servo aileron
    { eSERVO_2_ID, &gTimerBoardConfs[TIM5_CH2_IDX], 0, 50, 1.0F, 1.0F, 1.0F },
    // Servo elevator
    { eSERVO_3_ID, &gTimerBoardConfs[TIM5_CH3_IDX], 0, 50, 1.0F, 1.0F, 1.0F },
    // Servo rudder
    { eSERVO_4_ID, &gTimerBoardConfs[TIM5_CH4_IDX], 0, 50, 1.0F, 1.0F, 1.0F },
    // Right servo motor
    { eSERVO_5_ID, &gTimerBoardConfs[TIM8_CH3_IDX], 0, 50, 1.0F, 1.0F, 1.0F },
    // Right servo aileron
    { eSERVO_6_ID, &gTimerBoardConfs[TIM8_CH4_IDX], 0, 50, 1.0F, 1.0F, 1.0F },
    // Servo flight mode
    { eSERVO_7_ID, &gTimerBoardConfs[TIM12_CH1_IDX], 0, 50, 1.0F, 1.0F, 1.0F }
};

#define LEFT_SERVO_MOTOR_IDX  0
#define RIGHT_SERVO_MOTOR_IDX 4

static MotorBoardConf_t gMotorBoardConfs[] = {
    // Left motor
    { .motorId               = eMOTOR_1_ID,
      .pTimerBoardConf       = &gTimerBoardConfs[TIM12_CH1_IDX],
      .pLinkedServoBoardConf = &gServoBoardConfs[LEFT_SERVO_MOTOR_IDX],
      .useDMA                = 1,
      .dshotSpeed            = 150,
      .pidRollMix            = 1.0F,
      .pidPitchMix           = 1.0F,
      .pidYawMix             = 1.0F },
    // Right motor
    { .motorId               = eMOTOR_2_ID,
      .pTimerBoardConf       = &gTimerBoardConfs[TIM12_CH2_IDX],
      .pLinkedServoBoardConf = &gServoBoardConfs[RIGHT_SERVO_MOTOR_IDX],
      .useDMA                = 1,
      .dshotSpeed            = 150,
      .pidRollMix            = 1.0F,
      .pidPitchMix           = 1.0F,
      .pidYawMix             = 1.0F }
};

#define LEFT_MOTOR_IDX  0
#define RIGHT_MOTOR_IDX 1

static BoardConf_t gBoardConf = { 0 };

uint8_t BoardConfInit (void) {

    gBoardConf.pDeviceBoardConfs = gDeviceBoardConfs;
    gBoardConf.numDevices =
    sizeof (gDeviceBoardConfs) / sizeof (gDeviceBoardConfs[0]);

    gBoardConf.pServoBoardConfs = gServoBoardConfs;
    gBoardConf.numServos =
    sizeof (gServoBoardConfs) / sizeof (gServoBoardConfs[0]);

    gBoardConf.pMotorBoardConfs = gMotorBoardConfs;
    gBoardConf.numMotors =
    sizeof (gMotorBoardConfs) / sizeof (gMotorBoardConfs[0]);

    // Link left servo motor to left motor
    gServoBoardConfs[LEFT_SERVO_MOTOR_IDX].pLinkedMotorBoardConf =
    &gMotorBoardConfs[LEFT_MOTOR_IDX];

    // Link right servo motor to right motor
    gServoBoardConfs[RIGHT_SERVO_MOTOR_IDX].pLinkedMotorBoardConf =
    &gMotorBoardConfs[RIGHT_MOTOR_IDX];

    return 1;
}

BoardConf_t* BoardConfGet (void) {
    return &gBoardConf;
}

DeviceBoardConf_t* BoardConfGetDeviceById (eDEVICE_ID_t deviceId) {

    for (uint32_t i = 0; i < gBoardConf.numDevices; ++i) {
        if (gBoardConf.pDeviceBoardConfs[i].deviceId == deviceId) {
            return &gBoardConf.pDeviceBoardConfs[i];
        }
    }
    return 0;
}

ServoBoardConf_t* BoardConfGetServoById (eDEVICE_ID_t servoId) {

    for (uint32_t i = 0; i < gBoardConf.numServos; ++i) {
        if (gBoardConf.pServoBoardConfs[i].servoId == servoId) {
            return &gBoardConf.pServoBoardConfs[i];
        }
    }
    return 0;
}

MotorBoardConf_t* BoardConfGetMotorById (eDEVICE_ID_t motorId) {

    for (uint32_t i = 0; i < gBoardConf.numMotors; ++i) {
        if (gBoardConf.pMotorBoardConfs[i].motorId == motorId) {
            return &gBoardConf.pMotorBoardConfs[i];
        }
    }
    return 0;
}