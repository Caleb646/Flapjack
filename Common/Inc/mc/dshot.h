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
    TimerBoardConf_t timerBoardConf;
    MotorBoardConf_t motorBoardConf;
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

/* Functions */
eSTATUS_t DShotInit (DShotInitConf_t conf);
eSTATUS_t DShotStart (eDEVICE_ID_t deviceId);
eSTATUS_t DShotStop (eDEVICE_ID_t deviceId);
eSTATUS_t DShotWrite (eDEVICE_ID_t deviceId, uint16_t motorVal);

#define DSHOT_INIT_BITBANG(pSTATUS, MOTOR_BOARD_CONF, TIMER_BOARD_CONF) \
    do {                                                                \
        DShotInitConf_t conf = { 0 };                                   \
        conf.timerBoardConf  = (TIMER_BOARD_CONF);                      \
        conf.motorBoardConf  = (MOTOR_BOARD_CONF);                      \
        *(pSTATUS)           = DShotInit (conf);                        \
    } while (0)

#define DSHOT_INIT_DMA(pSTATUS, MOTOR_BOARD_CONF, TIMER_BOARD_CONF) \
    do {                                                            \
        DShotInitConf_t conf = { 0 };                               \
        conf.timerBoardConf  = (TIMER_BOARD_CONF);                  \
        conf.motorBoardConf  = (MOTOR_BOARD_CONF);                  \
        *(pSTATUS)           = DShotInit (conf);                    \
    } while (0)


#endif /* __MOTION_CONTROL_DSHOT_H__ */