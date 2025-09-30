#ifndef CONF_BASE_H
#define CONF_BASE_H

#include "conf/ids.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    eGPIO_ID_t id;
    eGPIO_PURPOSE_t purpose;
    uint16_t alternate;
} GPIOBoardConf_t;

typedef struct {
    eTIMER_ID_t timerId;
    GPIOBoardConf_t* pGPIOBoardConf;
} TimerBoardConf_t;

typedef struct {
    eBUS_ID_t busId;
} BusHeaderBoardConf_t;

typedef struct {
    eDEVICE_ID_t deviceId;
    GPIOBoardConf_t* pNssBoardConf;
} SPIDeviceMapping_t;

typedef struct {
    BusHeaderBoardConf_t header;
    GPIOBoardConf_t* pSckBoardConf;
    GPIOBoardConf_t* pMisoBoardConf;
    GPIOBoardConf_t* pMosiBoardConf;
    SPIDeviceMapping_t* pConnectedDevices; // Array of connected devices
    uint8_t numConnectedDevices;
    uint16_t speedKHz;
} SPIBoardConf_t;

typedef struct {
    BusHeaderBoardConf_t header;
    GPIOBoardConf_t* pTxBoardConf;
    GPIOBoardConf_t* pRxBoardConf;
    uint32_t baudRate;
} UARTBoardConf_t;

typedef struct {
    BusHeaderBoardConf_t header;
    GPIOBoardConf_t* pSclBoardConf;
    GPIOBoardConf_t* pSdaBoardConf;
} I2CBoardConf_t;

typedef struct {
    eEXTI_ID_t extiId;
    eGPIO_ID_t gpioId;
} EXTIBoardConf_t;

typedef struct {
    EXTIBoardConf_t* pExtiBoardConf;
    BusHeaderBoardConf_t* pBusBoardConf;
} GenericDeviceConf_t;

typedef struct DeviceBoardConf_s DeviceBoardConf_t;
typedef struct MotorDeviceConf_s MotorDeviceConf_t;

typedef struct ServoDeviceConf_s {
    TimerBoardConf_t* pTimerBoardConf;
    DeviceBoardConf_t* pLinkedMotorBoardConf;
    uint32_t pwmFrequency;
    float pidRollMix;
    float pidPitchMix;
    float pidYawMix;
} ServoDeviceConf_t;

typedef struct MotorDeviceConf_s {
    TimerBoardConf_t* pTimerBoardConf;
    DeviceBoardConf_t* pLinkedServoBoardConf;
    uint8_t dshotSpeed;
    float pidRollMix;
    float pidPitchMix;
    float pidYawMix;
} MotorDeviceConf_t;

typedef struct DeviceBoardConf_s {
    eDEVICE_ID_t deviceId;
    bool useDMA;
    union {
        ServoDeviceConf_t servo;
        MotorDeviceConf_t motor;
        GenericDeviceConf_t generic;
    };
} DeviceBoardConf_t;

typedef struct {
    DeviceBoardConf_t** ppDeviceBoardConfs;
    uint8_t numDevices;
} BoardConf_t;

#endif // CONF_BASE_H