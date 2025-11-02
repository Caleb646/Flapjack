#ifndef __MOTION_CONTROL_DSHOT_H__
#define __MOTION_CONTROL_DSHOT_H__

#include "common.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "conf/ids.h"
#include "hal.h"
#include "peripheral/dma.h"
#include "peripheral/gpio.h"
#include "peripheral/timer.h"

#define DSHOT_DMA_BUFFER_SIZE 18U /* resolution + frame reset (2us) */
#define DSHOT_FRAME_SIZE      16U
#define DSHOT_MIN_THROTTLE    48U
#define DSHOT_MAX_THROTTLE    2047U
#define DSHOT_RANGE           (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)

typedef uint8_t eDSHOT_TYPE_t;
enum { eDSHOT_TYPE_150 = 0, eDSHOT_TYPE_300 = 1, eDSHOT_TYPE_600 = 2 };

typedef uint8_t eDSHOT_OP_MODE_t;
enum { eDSHOT_OP_MODE_BB_NO_TIMER = 0, eDSHOT_OP_MODE_BB_WITH_TIMER = 1, eDSHOT_OP_MODE_DMA = 2 };

typedef struct DShot_s DShot_t;
typedef DShot_t vDShot_t;
typedef eSTATUS_t (*DShot_Write_Fn_t) (vDShot_t* pDShot, uint16_t motorVal);

typedef struct {
    eDEVICE_ID_t motorId;
    MotorDeviceConf_t motorBoardConf;
} DShotInitConf_t;

typedef struct DShot_s {
    eDSHOT_TYPE_t dshotType;
    eDSHOT_OP_MODE_t opMode;
    eDEVICE_ID_t deviceId;

    union {
        vTimer_t* pTimer;
        vIO_t* pGPIO;
    };

    uint32_t pMotorDmaBuffer[DSHOT_DMA_BUFFER_SIZE];

    uint16_t timerTicksPeriod;
    uint16_t timerTicksforBit_1;
    uint16_t timerTicksforBit_0;

    float usPeriod;
    float usValforBit_1;
    float usValforBit_0;

    uint32_t cyclesPeriod;
    uint32_t cyclesforBit_1;
    uint32_t cyclesforBit_0;

    DShot_Write_Fn_t writeFn;
    bool isInitialized;
} DShot_t;

// typedef DShot_t volatile vDShot_t;
typedef DShot_t vDShot_t;

vDShot_t* DShotGetById (eDEVICE_ID_t deviceId);
eSTATUS_t DShotInit (DShotInitConf_t conf, DShot_t* pOutDShot);
eSTATUS_t DShotStart (vDShot_t* pDShot);
eSTATUS_t DShotStop (vDShot_t* pDShot);
eSTATUS_t DShotWrite (vDShot_t* pDShot, uint16_t motorVal);

#define DSHOT_INIT(pSTATUS, DEVICE_BOARD_CONF)               \
    do {                                                     \
        DShotInitConf_t conf = { 0 };                        \
        conf.motorId         = (DEVICE_BOARD_CONF).deviceId; \
        conf.motorBoardConf  = (DEVICE_BOARD_CONF).motor;    \
        *(pSTATUS)           = DShotInit (conf, NULL);       \
    } while (0)

#define DSHOT_START(MOTOR_ID)        DShotStart (DShotGetById (MOTOR_ID))
#define DSHOT_STOP(MOTOR_ID)         DShotStop (DShotGetById (MOTOR_ID))
#define DSHOT_WRITE(MOTOR_ID, VALUE) DShotWrite (DShotGetById (MOTOR_ID), VALUE)


#endif /* __MOTION_CONTROL_DSHOT_H__ */