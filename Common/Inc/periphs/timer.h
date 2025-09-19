#ifndef __PERIPHS_TIMER_H
#define __PERIPHS_TIMER_H

#include "common.h"
#include "conf.h"
#include "dma.h"
#include "hal.h"
#include "log/logger.h"
#include <stdint.h>

#define PWM_CHECK_OK(pPwmHandle) \
    ((pPwmHandle) != NULL && (pPwmHandle)->timer.Instance != NULL)

#define PWM_CHECK_CONF_OK(pPwmConf) \
    ((pPwmConf) != NULL && (pPwmConf)->base.pTimer != NULL)

#define PWMDMA_CHECK_CONF_OK(pPwmConf) \
    ((pPwmConf) != NULL && (pPwmConf)->base.pTimer != NULL)

#define PWM_MHZ2HZ(x) ((x) * 1000000U)
#define PWM_HZ2US(hz) \
    ((float)(1000000.0F /* <-- prescaled clock, 1 MHz */ / (float)(hz))) // Convert Hz to microseconds

#define PWM_US2HZ(us) \
    ((float)(1000000.0F /* <-- 1 second in us */ / (float)(us))) // Convert microseconds to Hz

#define PWM_US2DC(us, usPeriod) \
    (((float)(us) * (100.0F / (float)(usPeriod)))) // Convert microseconds to duty cycle percentage

#define PWM_SET_PRESCALER(pPwmHandle, valPrescaler)          \
    do {                                                     \
        (pPwmHandle)->timer.Instance->PSC  = (valPrescaler); \
        (pPwmHandle)->timer.Init.Prescaler = (valPrescaler); \
    } while (0)

#define PWM_SET_PERIOD(pPwmHandle, valPeriod)            \
    do {                                                 \
        (pPwmHandle)->timer.Instance->ARR = (valPeriod); \
        (pPwmHandle)->timer.Init.Period   = (valPeriod); \
    } while (0)

#define PWM_SET_COMPARE(pPwmHandle, compare)                   \
    do {                                                       \
        if ((pPwmHandle)->channelID == TIM_CHANNEL_1) {        \
            (pPwmHandle)->timer.Instance->CCR1 = (compare);    \
        } else if ((pPwmHandle)->channelID == TIM_CHANNEL_2) { \
            (pPwmHandle)->timer.Instance->CCR2 = (compare);    \
        } else if ((pPwmHandle)->channelID == TIM_CHANNEL_3) { \
            (pPwmHandle)->timer.Instance->CCR3 = (compare);    \
        } else if ((pPwmHandle)->channelID == TIM_CHANNEL_4) { \
            (pPwmHandle)->timer.Instance->CCR4 = (compare);    \
        } else if ((pPwmHandle)->channelID == TIM_CHANNEL_5) { \
            (pPwmHandle)->timer.Instance->CCR5 = (compare);    \
        } else if ((pPwmHandle)->channelID == TIM_CHANNEL_6) { \
            (pPwmHandle)->timer.Instance->CCR6 = (compare);    \
        } else {                                               \
            LOG_ERROR (                                        \
            "Invalid timer channel ID: 0x%X",                  \
            (uint16_t)(pPwmHandle)->channelID                  \
            );                                                 \
        }                                                      \
    } while (0)

#define PWM_CREATE_CONF(PTR_TIMER, CHANNEL_ID, HZ_PERIOD, AUTO_RELOAD) \
    {                                                                  \
        .base = {                                                      \
            .pTimer       = (PTR_TIMER),                               \
            .channelID    = (CHANNEL_ID),                              \
            .hzPeriod     = (HZ_PERIOD),                               \
            .doAutoReload = (AUTO_RELOAD)                              \
        }                                                              \
    }

#define TIMER_MAX_NUMBER_OF_HANDLES  4U
#define TIMER_MAX_NUMBER_OF_CHANNELS 4U

typedef enum {
    eTIM_DMA_ID_UPDATE = TIM_DMA_ID_UPDATE, /*!< Index of the DMA handle used for Update DMA requests */
    eTIM_DMA_ID_CC1 = TIM_DMA_ID_CC1, /*!< Index of the DMA handle used for Capture/Compare 1 DMA requests */
    eTIM_DMA_ID_CC2 = TIM_DMA_ID_CC2, /*!< Index of the DMA handle used for Capture/Compare 2 DMA requests */
    eTIM_DMA_ID_CC3 = TIM_DMA_ID_CC3, /*!< Index of the DMA handle used for Capture/Compare 3 DMA requests */
    eTIM_DMA_ID_CC4 = TIM_DMA_ID_CC4, /*!< Index of the DMA handle used for Capture/Compare 4 DMA requests */
    eTIM_DMA_ID_COMMUTATION = TIM_DMA_ID_COMMUTATION, /*!< Index of the DMA handle used for Commutation DMA requests */
    eTIM_DMA_ID_TRIGGER = TIM_DMA_ID_TRIGGER /*!< Index of the DMA handle used for Trigger DMA requests */
} eTIM_DMA_REGIDX;

// typedef struct {
//     TIM_TypeDef* pTimer;
//     uint32_t channelID;
//     uint32_t hzPeriod; // PWM period frequency in Hz
//     uint8_t doAutoReload;
// } PWMBaseConfig;

typedef uint8_t eTIMER_MODE_t;
enum {
    eTIMER_MODE_PWM = 0,
    // eTIMER_MODE_INPUT_CAPTURE,
    // eTIMER_MODE_OUTPUT_COMPARE,
    // eTIMER_MODE_ONE_PULSE,
    // eTIMER_MODE_ENCODER
};

typedef struct {
    eDEVICE_ID_t deviceId;
    eTIMER_MODE_t mode;
    eTIMER_ID_t timerId;
    uint32_t hzPeriod; // PWM period frequency in Hz
    BOOL_t usingDMA;
    BOOL_t doAutoPreload;
} TimerInitConf_t;

typedef struct {
    eDEVICE_ID_t deviceId;
    eTIMER_ID_t timerId;
    eTIMER_MODE_t mode;
    BOOL_t usingDMA;
    BOOL_t isChannelInitialized;
} TimerChannel_t;

typedef struct {
    TIM_HandleTypeDef handle;
    TimerChannel_t channels[TIMER_MAX_NUMBER_OF_CHANNELS];
    BOOL_t isTimerInitialized;
} Timer_t;

// typedef struct {
//     PWMBaseConfig base;
// } PWMConfig;

// typedef struct {
//     PWMBaseConfig base;
//     uint16_t dmaRegIDXs[7];  // DMA register indices for the timer
//     channels uint16_t dmaRegIDXCount; // Number of DMA register indices
// } PWM_DMAConfig;

// typedef struct {
//     TIM_HandleTypeDef timer;
//     uint32_t channelID;
// } PWMHandle;

// typedef struct {
//     PWMHandle pwm;
//     DMA_HandleTypeDef* pDMA;
// } PWM_DMAHandle;

typedef enum {
    ePWM_DMA_CB_TRANSFER_COMPLETE = 0,
    ePWM_DMA_CB_HALF_TRANSFER     = 1,
    ePWM_DMA_CB_TRANSFER_ERROR    = 2,
    ePWM_DMA_CB_ABORT             = 3
} ePWM_DMA_CB_TYPE;

typedef void (*PWM_DMACallback) (TIM_HandleTypeDef* htim);


eSTATUS_t TimerInit (TimerInitConf_t const* pConf);

eSTATUS_t PWMInit (PWMConfig config, PWMHandle* pOutHandle);
eSTATUS_t PWMStart (PWMHandle* pHandle);
eSTATUS_t PWMStop (PWMHandle* pHandle);
eSTATUS_t PWMWrite (PWMHandle* pHandle, uint32_t usUpTime);

eSTATUS_t PWM_DMAInit (PWM_DMAConfig timConfig, DMAConfig dmaConfig, PWM_DMAHandle* pOutHandle);
eSTATUS_t PWM_DMAStart (PWM_DMAHandle* pHandle, uint32_t const* pData, uint16_t Length);
eSTATUS_t
PWM_DMARegisterCallback (PWM_DMAHandle* pHandle, uint32_t channelID, PWM_DMACallback callback, ePWM_DMA_CB_TYPE cbType);


#endif // __PERIPHS_TIMER_H