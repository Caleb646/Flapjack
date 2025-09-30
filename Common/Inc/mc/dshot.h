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

typedef struct {
    eDEVICE_ID_t motorId;
    MotorDeviceConf_t motorBoardConf;
} DShotInitConf_t;

typedef struct {
    eDSHOT_TYPE_t dshotType;
    eDEVICE_ID_t deviceId;
    eTIMER_ID_t timerId;
    bool usingDMA;
    vIO_t* pGPIO; /*!< GPIO handle for bitbang */
    uint32_t pMotorDmaBuffer[DSHOT_DMA_BUFFER_SIZE]; /*!< DMA buffer for DShot */

    uint16_t timerTicksPeriod;
    uint16_t timerTicksforBit_1; /*!< microsecond value to send a 1 */
    uint16_t timerTicksforBit_0; /*!< microsecond value to send a 0 */

    float usPeriod;      /*!< microsecond value for one bit period */
    float usValforBit_1; /*!< microsecond value to send a 1 */
    float usValforBit_0; /*!< microsecond value to send a 0 */

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

#define DSHOT_START(MOTOR_ID) DShotStart (DShotGetById (MOTOR_ID))
#define DSHOT_STOP(MOTOR_ID)  DShotStop (DShotGetById (MOTOR_ID))
#define DSHOT_WRITE(MOTOR_ID, VALUE) \
    DShotWrite (DShotGetById (MOTOR_ID), VALUE)


#endif /* __MOTION_CONTROL_DSHOT_H__ */