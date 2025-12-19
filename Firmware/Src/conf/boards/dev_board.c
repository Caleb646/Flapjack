#include "common.h"
#include "conf/board.h"
#include "conf/ids.h"
#include "hal.h"
#include <stdbool.h>
#include <stdint.h>

static DevDesc_t leftMotor = {
    .deviceId   = eMOTOR_1_ID,
    .isRequired = true,
    .actDev   = {
        .pLinkedDesc   = NULL,
        .timerDesc    = TIM_DESC_CREATE (TIMER_ID_MAKE (eTIMER_8_DEVICE_ID, eTIMER_CHANNEL_1_ID), GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_6), GPIO_AF3_TIM8),
        .pidDesc       = PID_DESC_ANGLE_CREATE (1.0F, 1.0F, 1.0F),
        .protDesc  = ACTPROT_DESC_DSHOT_CREATE (eDSHOT_TYPE_TIMER_ONLY, eDSHOT_SPEED_150),
    },
};

static DevDesc_t leftServo = {
    .deviceId   = eSERVO_1_ID,
    .isRequired = true,
    .actDev   = {
        .pLinkedDesc   = NULL,
        .timerDesc    = TIM_DESC_CREATE (TIMER_ID_MAKE (eTIMER_13_DEVICE_ID, eTIMER_CHANNEL_1_ID), GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_8), GPIO_AF9_TIM13),
        .pidDesc       = PID_DESC_ANGLE_CREATE (1.0F, 1.0F, 1.0F),
        .protDesc  = ACTPROT_DESC_PWM_CREATE (50U),
    },
};

static BusDesc_t spi2 = {
    .busId = eSPI_2_BUS_ID,
    .spiDesc = {
        .speed = eSPI_SPEED_500KHZ,
        .sckDesc = GPIO_DESC_CREATE_SPI_DPIN (GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_12), GPIO_AF5_SPI2),
        .misoDesc = GPIO_DESC_CREATE_SPI_DPIN (GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_2), GPIO_AF5_SPI2),
        .mosiDesc = GPIO_DESC_CREATE_SPI_DPIN (GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_3), GPIO_AF5_SPI2),
        .connectedDevices = {
            { .deviceId = eIMU_DEVICE_ID, .nssDesc = GPIO_DESC_CREATE_SPI_NSS (GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_11)) },
        },
        .nConnectedDevices = 1,
    }
};

static DevDesc_t imu = {
    .deviceId = eIMU_DEVICE_ID,
    .isRequired = true,
    .flags = 0U,
    .genDev = {
        .pBusDesc = &spi2,
        .extiDesc = { // TODO: set correct EXTI
            .extiId = eEXTI_11_ID,
            .gpioDesc = GPIO_DESC_CREATE (GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_11), GPIO_MODE_IT_RISING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0),
        },
    }
};

static BusDesc_t uart1 = { .busId    = eUART_1_BUS_ID,
                           .uartDesc = {
                           .txDesc = GPIO_DESC_CREATE_UART_DPIN (GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_10), GPIO_AF7_USART1),
                           .rxDesc = GPIO_DESC_CREATE_UART_DPIN (GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_9), GPIO_AF7_USART1),
                           .baudRate = eUART_BAUDRATE_230400,
                           } };

static DevDesc_t serialDebug = {
    .deviceId = eSERIAL_DEBUG_DEVICE_ID,
    .isRequired = true,
    .flags = 0U,
    .genDev = {
        .pBusDesc = &uart1,
        .extiDesc = {
            .extiId = eEXTI_ID_NULL,
        },
    }
};

static DevDesc_t* g_DeviceDescs[] = { &imu, &serialDebug, &leftServo, &leftMotor };

eSTATUS_t DeviceTree_InitImpl (DeviceTree_t* pOutDeviceTree) {

    // link left motor to left servo
    DEV_DESC_GET_ACTDEV_LINKED (&leftMotor) = &leftServo;
    // link left servo to left motor
    DEV_DESC_GET_ACTDEV_LINKED (&leftServo) = &leftMotor;

    pOutDeviceTree->ppDeviceDescs = g_DeviceDescs;
    pOutDeviceTree->numDevices    = sizeof (g_DeviceDescs) / sizeof (g_DeviceDescs[0]);
    return eSTATUS_OK;
}