#ifndef CONF_BASE_H
#define CONF_BASE_H

#include "conf/ids.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t mode;
    uint32_t pull;
    uint32_t speed;
    uint8_t alternate;
} GPIOSharedConf_t;

typedef struct {
    eGPIO_ID_t id;
    // union {
    GPIOSharedConf_t const* pShared;
    GPIOSharedConf_t conf;
    // };
} GPIOBoardConf_t;

typedef struct {
    eTIMER_ID_t timerId;
    GPIOBoardConf_t const* pGPIOBoardConf;
} TimerBoardConf_t;

typedef struct {
    eDEVICE_ID_t deviceId;
    GPIOBoardConf_t const* pNssBoardConf;
} SPIDeviceMapping_t;

typedef struct {
    GPIOBoardConf_t const* pSckBoardConf;
    GPIOBoardConf_t const* pMisoBoardConf;
    GPIOBoardConf_t const* pMosiBoardConf;
    SPIDeviceMapping_t const* pConnectedDevices;
    uint8_t numConnectedDevices;
    uint16_t speedKHz;
} SPIBoardConf_t;

typedef struct {
    GPIOBoardConf_t const* pTxBoardConf;
    GPIOBoardConf_t const* pRxBoardConf;
    uint32_t baudRate;
} UARTBoardConf_t;

typedef struct {
    GPIOBoardConf_t const* pSclBoardConf;
    GPIOBoardConf_t const* pSdaBoardConf;
} I2CBoardConf_t;

typedef struct {
    eBUS_ID_t busId;
    union {
        I2CBoardConf_t I2CBoardConf;
        SPIBoardConf_t SPIBoardConf;
        UARTBoardConf_t UARTBoardConf;
    };
} BusBoardConf_t;

typedef struct {
    eEXTI_ID_t extiId;
    GPIOBoardConf_t const* pGPIOBoardConf;
} EXTIBoardConf_t;

typedef struct {
    EXTIBoardConf_t* pExtiBoardConf;
    BusBoardConf_t* pBusBoardConf;
    bool useDMAForWrites;
    bool useDMAForReads;
    bool useInterruptsForWrites;
    bool useInterruptsForReads;
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
    bool useDMA;
    eDSHOT_TYPE_t dshotSpeed;
    float pidRollMix;
    float pidPitchMix;
    float pidYawMix;
} MotorDeviceConf_t;

typedef struct DeviceBoardConf_s {
    eDEVICE_ID_t deviceId;
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

extern bool ge_isBoardConfInitialized;
extern BoardConf_t ge_BoardConf;

bool BoardConfInit_DevBoard (void);
bool BoardConfInit_MyBoard (void);
BoardConf_t* BoardConfGet (void);
DeviceBoardConf_t* BoardConfGetDeviceById (eDEVICE_ID_t deviceId);

#endif // CONF_BASE_H