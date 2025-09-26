#ifndef __PERIPHS_TIMER_H
#define __PERIPHS_TIMER_H

#include "common.h"
#include "conf/conf.h"
#include "hal.h"
#include "log/logger.h"
#include "peripheral/dma.h"
#include <stdint.h>


#define PWM_MHZ2HZ(x) ((x) * 1000000U)
#define PWM_HZ2US(hz) \
    ((float)(1000000.0F /* <-- prescaled clock, 1 MHz */ / (float)(hz))) // Convert Hz to microseconds

#define PWM_US2HZ(us) \
    ((float)(1000000.0F /* <-- 1 second in us */ / (float)(us))) // Convert microseconds to Hz

#define PWM_US2DC(us, usPeriod) \
    (((float)(us) * (100.0F / (float)(usPeriod)))) // Convert microseconds to duty cycle percentage

#define TIMER_NTIMERS   eTIMER_ID_MAX
#define TIMER_NCHANNELS eTIMER_CHANNEL_ID_MAX

typedef uint8_t eTIMER_SET_REG_t;
enum {
    eTIMER_SET_REG_PRESCALER = 0,
    eTIMER_SET_REG_PERIOD,
    eTIMER_SET_REG_COMPARE,
};

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

typedef struct {
    eDEVICE_ID_t deviceId;
    eTIMER_MODE_t mode;
    eTIMER_ID_t timerId;
    uint32_t hzPeriod; // PWM period frequency in Hz
    BOOL_t usingDMA;
    BOOL_t doAutoPreload;
} TimerInitConf_t;

typedef void (*TimerCallback_t) (eTIMER_ID_t timerId);

typedef struct {
    TimerInitConf_t conf;
    BOOL_t isChannelInitialized;
    TimerCallback_t errorCallback;
    TimerCallback_t pulseFinishedCallback;
} TimerChannel_t;

typedef struct {
    TIM_HandleTypeDef handle;
    TimerChannel_t channels[TIMER_NCHANNELS];
    BOOL_t isTimerInitialized;
} Timer_t;

typedef Timer_t volatile vTimer_t;

typedef uint8_t eTIMER_CALLBACK_ID_t;
enum {
    eTIMER_CALLBACK_TRANSFER_COMPLETE = 0,
    eTIMER_CALLBACK_TRANSFER_ERROR,
};

eSTATUS_t TimerInit (TimerInitConf_t conf);
eSTATUS_t TimerStart (eTIMER_ID_t timerId, uint32_t const* pData, uint16_t Length);
eSTATUS_t TimerStop (eTIMER_ID_t timerId);
eSTATUS_t TimerWrite (eTIMER_ID_t timerId, uint32_t usUpTime);
eSTATUS_t TimerRegisterCallback (eTIMER_ID_t timerId, TimerCallback_t callback, eTIMER_CALLBACK_ID_t cbType);
eSTATUS_t TimerSetRegister (eTIMER_ID_t timerId, eTIMER_SET_REG_t regType, uint32_t value);

#define TIMER_SET_PRESCALER(TIMER_ID, PRESCALER) \
    TimerSetRegister ((TIMER_ID), eTIMER_SET_REG_PRESCALER, (PRESCALER))
#define TIMER_SET_PERIOD(TIMER_ID, PERIOD) \
    TimerSetRegister ((TIMER_ID), eTIMER_SET_REG_PERIOD, (PERIOD))
#define TIMER_SET_COMPARE(TIMER_ID, COMPARE) \
    TimerSetRegister ((TIMER_ID), eTIMER_SET_REG_COMPARE, (COMPARE))


#define TIMER_INIT_PWM(pSTATUS, DEVICE_ID, TIMER_ID, HZ, DO_AUTO_PRELOAD) \
    do {                                                                  \
        TimerInitConf_t conf = { 0 };                                     \
        conf.deviceId        = (DEVICE_ID);                               \
        conf.mode            = eTIMER_MODE_PWM;                           \
        conf.timerId         = (TIMER_ID);                                \
        conf.hzPeriod        = (HZ);                                      \
        conf.usingDMA        = FALSE;                                     \
        conf.doAutoPreload   = (DO_AUTO_PRELOAD);                         \
        *(pSTATUS)           = TimerInit (conf);                          \
    } while (0)


#define TIMER_INIT_PWM_DMA(pSTATUS, DEVICE_ID, TIMER_ID) \
    do {                                                 \
        TimerInitConf_t conf = { 0 };                    \
        conf.deviceId        = (DEVICE_ID);              \
        conf.mode            = eTIMER_MODE_PWM;          \
        conf.timerId         = (TIMER_ID);               \
        conf.hzPeriod        = 0U;                       \
        conf.usingDMA        = TRUE;                     \
        conf.doAutoPreload   = TRUE;                     \
        *(pSTATUS)           = TimerInit (conf);         \
    } while (0)


#endif // __PERIPHS_TIMER_H