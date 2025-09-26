#ifndef CONF_BASE_H
#define CONF_BASE_H

#include "conf/ids.h"
#include <stdint.h>

typedef struct {
    eTIMER_ID_t timerId;
    eGPIO_ID_t gpioId;
    uint16_t gpioAlternate;
} TimerBoardConf_t;

typedef struct {
    eBUS_ID_t busId;
} BusHeaderBoardConf_t;

typedef struct {
    BusHeaderBoardConf_t header;
    eGPIO_ID_t sckId;
    eGPIO_ID_t misoId;
    eGPIO_ID_t mosiId;
    uint16_t gpioAlternate;
    uint16_t speedKHz;
} SPIBoardConf_t;

typedef struct {
    BusHeaderBoardConf_t header;
    eGPIO_ID_t txId;
    eGPIO_ID_t rxId;
    uint16_t gpioAlternate;
    uint32_t baudRate;
} UARTBoardConf_t;

typedef struct {
    BusHeaderBoardConf_t header;
    eGPIO_ID_t sclId;
    eGPIO_ID_t sdaId;
    uint16_t gpioAlternate;
} I2CBoardConf_t;

typedef struct {
    eEXTI_ID_t extiId;
    eGPIO_ID_t gpioId;
} EXTIBoardConf_t;

typedef struct {
    eDEVICE_ID_t deviceId;
    EXTIBoardConf_t extiBoardConf;
    BusHeaderBoardConf_t* pBusBoardConf;
    eGPIO_ID_t nssId;
    uint8_t useDMARead;
    uint8_t useDMAWrite;
} DeviceBoardConf_t;

// typedef struct {
//     DeviceBoardConf_t base;


// } IMUDeviceBoardConf_t;

struct MotorBoardConf_t;

typedef struct {
    eDEVICE_ID_t servoId;
    TimerBoardConf_t* pTimerBoardConf;
    struct MotorBoardConf_t* pLinkedMotorBoardConf;
    uint32_t pwmFrequency;
    float pidRollMix;
    float pidPitchMix;
    float pidYawMix;
} ServoBoardConf_t;

typedef struct {
    eDEVICE_ID_t motorId;
    TimerBoardConf_t* pTimerBoardConf;
    ServoBoardConf_t* pLinkedServoBoardConf;
    uint8_t useDMA;
    uint8_t dshotSpeed;
    float pidRollMix;
    float pidPitchMix;
    float pidYawMix;
} MotorBoardConf_t;

typedef struct {
    DeviceBoardConf_t* pDeviceBoardConfs;
    uint8_t numDevices;
    ServoBoardConf_t* pServoBoardConfs;
    uint8_t numServos;
    MotorBoardConf_t* pMotorBoardConfs;
    uint8_t numMotors;
} BoardConf_t;

#endif // CONF_BASE_H