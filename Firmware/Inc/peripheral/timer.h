#ifndef __PERIPHS_TIMER_H
#define __PERIPHS_TIMER_H

#include "conf/conf.h"
#include "core/core.h"
#include "core/log/logger.h"
#include "hal.h"
#include "peripheral/dma.h"
#include <stdint.h>



typedef uint8_t eTIMER_MODE_t;
enum {
    eTIMER_MODE_PWM = 0,
    // eTIMER_MODE_INPUT_CAPTURE,
    // eTIMER_MODE_OUTPUT_COMPARE,
    // eTIMER_MODE_ONE_PULSE,
    // eTIMER_MODE_ENCODER
};

typedef uint8_t eTIMER_CHANNEL_STATE_t;
enum {
    eTIMER_CHANNEL_STATE_RESET = 0,
    eTIMER_CHANNEL_STATE_READY,
    eTIMER_CHANNEL_STATE_BUSY,
    eTIMER_CHANNEL_STATE_TIMEOUT,
    eTIMER_CHANNEL_STATE_ERROR
};

typedef uint8_t eTIMER_CALLBACK_ID_t;
enum {
    eTIMER_CALLBACK_TRANSFER_COMPLETE = 0,
    eTIMER_CALLBACK_TRANSFER_ERROR,
};

typedef struct {
    eDEVICE_ID_t deviceId;
    eTIMER_MODE_t mode;
    uint32_t hzPeriod;
    bool usingDMA;
    bool doAutoPreload;
    TimerBoardConf_t timerBoardConf;
} TimerInitConf_t;

typedef void (*TimerCallback_t) (eTIMER_ID_t timerId);

typedef struct {
    TIM_HandleTypeDef handle;
    bool isDeviceInitialized;
} TimerDevice_t;

typedef struct Timer_s {
    eDEVICE_ID_t deviceId;
    eTIMER_ID_t timerId;
    eTIMER_MODE_t mode;
    TimerDevice_t* pDevice;
    TimerCallback_t errorCallback;
    TimerCallback_t pulseFinishedCallback;
    bool usingDMA;
    bool isTimerInitialized;
} Timer_t;

// typedef Timer_t volatile vTimer_t;
typedef TimerDevice_t vTimerDevice_t;
typedef Timer_t vTimer_t;

// clang-format off
vTimer_t* Timer_Get_ById (eTIMER_ID_t timerId);
eSTATUS_t Timer_Init (TimerInitConf_t conf, Timer_t* pOutTimer);
eSTATUS_t Timer_Start (Timer_t* pTimer, uint32_t const* pData, uint16_t Length);
eSTATUS_t Timer_Stop (Timer_t* pTimer);
eSTATUS_t Timer_Write (Timer_t* pTimer, uint32_t usUpTime);
eSTATUS_t Timer_RegisterCallback (Timer_t* pTimer, TimerCallback_t callback, eTIMER_CALLBACK_ID_t cbType);
eTIMER_CHANNEL_STATE_t Timer_GetChannelState (Timer_t* pTimer);

#define TIMER_GET_DEVICE(pTIMER)   ((pTIMER)->pDevice)
#define TIMER_GET_HANDLE(pTIMER)   (&(TIMER_GET_DEVICE (pTIMER)->handle))
#define TIMER_GET_INSTANCE(pTIMER) (TIMER_GET_HANDLE (pTIMER)->Instance)
#define TIMER_GET_ID(pTIMER)       ((pTIMER)->timerId)
#define TIMER_GET_CHANNEL(pTIMER)  (TIMER_ID_TO_CHAN_IDX (TIMER_GET_ID(pTIMER)))

#define TIMER_SET_PRESCALER(pTIMER, PRESCALER) (TIMER_GET_INSTANCE (pTIMER)->PSC = (PRESCALER))
#define TIMER_SET_PERIOD(pTIMER, PERIOD) (TIMER_GET_INSTANCE (pTIMER)->ARR = (PERIOD))
#define TIMER_SET_COMPARE(pTIMER, COMPARE) (__HAL_TIM_SET_COMPARE (TIMER_GET_HANDLE (pTIMER), TIMER_ID_TO_HAL_CHAN (TIMER_GET_ID (pTIMER)), (COMPARE)))

#define TIMER_GET_INT_FLAG_REG(pTIMER) (&(TIMER_GET_INSTANCE (pTIMER)->SR))
#define TIMER_GET_CCIF_INT_FLAG_MASK(CHAN) ((CHAN) == eTIMER_CHANNEL_1_ID ? TIM_FLAG_CC1 : \
                                       (CHAN) == eTIMER_CHANNEL_2_ID ? TIM_FLAG_CC2 : \
                                       (CHAN) == eTIMER_CHANNEL_3_ID ? TIM_FLAG_CC3 : \
                                       (CHAN) == eTIMER_CHANNEL_4_ID ? TIM_FLAG_CC4 : 0)
#define TIMER_GET_CCIF_INT_FLAG(pTIMER) (__HAL_TIM_GET_FLAG (TIMER_GET_HANDLE (pTIMER), TIMER_GET_CCIF_INT_FLAG_MASK (TIMER_GET_CHANNEL(pTIMER))))
#define TIMER_CLR_CCIF_INT_FLAG(pTIMER) (__HAL_TIM_CLEAR_FLAG (TIMER_GET_HANDLE (pTIMER), TIMER_GET_CCIF_INT_FLAG_MASK (TIMER_GET_CHANNEL(pTIMER))))

#define TIMER_INIT(pSTATUS, DEVICE_ID, MODE, HZ, USING_DMA, DO_AUTO_PRELOAD, TIM_BOARD_CONF) \
    do {                                                                                     \
        TimerInitConf_t conf = { 0 };                                                        \
        conf.deviceId        = (DEVICE_ID);                                                  \
        conf.mode            = (MODE);                                                       \
        conf.hzPeriod        = (HZ);                                                         \
        conf.usingDMA        = (USING_DMA);                                                  \
        conf.timerBoardConf  = (TIM_BOARD_CONF);                                             \
        conf.doAutoPreload   = (DO_AUTO_PRELOAD);                                            \
        *(pSTATUS)           = Timer_Init (conf, NULL);                                      \
    } while (0)

#define TIMER_INIT_PWM(pSTATUS, DEVICE_ID, TIMER_ID, HZ, TIM_BOARD_CONF) TIMER_INIT ((pSTATUS), (DEVICE_ID), eTIMER_MODE_PWM, (HZ), false, true, (TIM_BOARD_CONF))
#define TIMER_INIT_PWM_DMA(pSTATUS, DEVICE_ID, TIMER_ID, TIM_BOARD_CONF) TIMER_INIT ((pSTATUS), (DEVICE_ID), eTIMER_MODE_PWM, 0U, true, true, (TIM_BOARD_CONF))
// #define TIMER_START(pTIMER, pDATA, SIZE) Timer_Start (pTIMER, pDATA, SIZE)
// #define TIMER_STOP(pTIMER) Timer_Stop (pTIMER)
// #define TIMER_WRITE(pTIMER, US_UPTIME) Timer_Write (pTIMER, US_UPTIME)
// #define TIMER_REGISTER_CALLBACK(pTIMER, CALLBACK, CB_TYPE) Timer_RegisterCallback (pTIMER, CALLBACK, CB_TYPE)
// #define TIMER_GET_CHANNEL_STATE(pTIMER) Timer_GetChannelState (pTIMER)
// clang-format on


#endif // __PERIPHS_TIMER_H