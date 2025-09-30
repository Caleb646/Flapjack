#include "peripheral/timer.h"
#include "common.h"
#include "conf/conf.h"
#include "hal.h"
#include "log/logger.h"
#include "mem/mem.h"
#include "peripheral/dma.h"
#include "peripheral/gpio.h"
#include <stdint.h>
#include <string.h>


static SHARED_MEM_SECTION vTimer_t gTimers[TIMER_NTIMERS] = { 0 };

static vTimer_t* TimerGetByInstance (TIM_TypeDef* instance);
static void TimerHandleCallback (TIM_HandleTypeDef* htim, eTIMER_CALLBACK_ID_t callbackType);
static eSTATUS_t
TimerDMAInit (vTimer_t* pTimer, TimerChannel_t* pChannel, TimerInitConf_t conf);

/*
 * The callback set by HAL_PWM_DMA_START call this function for Timer DMA transfer error callback.
 */
void HAL_TIM_ErrorCallback (TIM_HandleTypeDef* htim) {
    TimerHandleCallback (htim, eTIMER_CALLBACK_TRANSFER_ERROR);
}

/*
 * The callback set by HAL_PWM_DMA_START call this function for Timer DMA transfer half complete callback.
 */
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback (TIM_HandleTypeDef* htim) {

    if (htim == NULL) {
        return;
    }
}

/*
 * The callback set by HAL_PWM_DMA_START call this function for Timer DMA transfer complete callback.
 */
void HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef* htim) {

    TimerHandleCallback (htim, eTIMER_CALLBACK_TRANSFER_COMPLETE);
}

static void TimerHandleCallback (TIM_HandleTypeDef* htim, eTIMER_CALLBACK_ID_t callbackType) {

    if (htim == NULL) {
        return;
    }

    vTimer_t* pTimer = TimerGetByInstance (htim->Instance);
    if (pTimer == NULL) {
        return;
    }

    uint32_t activeChannel = htim->Channel;
    if (((int32_t)activeChannel - 1) < 0) {
        return;
    }

    TimerChannel_t* pChannel = &pTimer->channels[activeChannel - 1];
    if (pChannel == NULL) {
        return;
    }

    eTIMER_ID_t timerId = pChannel->timerId;

    switch (callbackType) {
    case eTIMER_CALLBACK_TRANSFER_COMPLETE:
        if (pChannel->pulseFinishedCallback != NULL) {
            pChannel->pulseFinishedCallback (timerId);
        }
        break;
    case eTIMER_CALLBACK_TRANSFER_ERROR:
        if (pChannel->errorCallback != NULL) {
            pChannel->errorCallback (timerId);
        }
        break;
    default: break;
    }
}

static vTimer_t* TimerGetByInstance (TIM_TypeDef* instance) {

    if (instance == NULL) {
        return NULL;
    }

    for (uint32_t i = 0; i < TIMER_NTIMERS; ++i) {
        if (gTimers[i].handle.Instance == instance) {
            return &gTimers[i];
        }
    }
    return NULL;
}

static TIM_TypeDef* TimerGetInstanceById (eTIMER_ID_t timerId) {

    // Clear channel bits
    switch (TIMER_ID_CLEAR_CHANNEL_BITS (timerId)) {
    case eTIMER_5_DEV_ID: return TIM5;
    case eTIMER_8_DEV_ID: return TIM8;
    case eTIMER_12_DEV_ID: return TIM12;
    case eTIMER_13_DEV_ID: return TIM13;
    default:
        LOG_ERROR ("Invalid timer ID: %u", (uint16_t)timerId);
        return NULL;
    }
}

vTimer_t* TimerGetById (eTIMER_ID_t timerId) {

    uint32_t timerIdx = TIMER_ID2IDX (timerId);
    if (timerIdx >= TIMER_NTIMERS) {
        LOG_ERROR ("Invalid timer index: %u", (uint16_t)timerIdx);
        return NULL;
    }
    return &gTimers[timerIdx];
}

TimerChannel_t* TimerGetChannelById (eTIMER_ID_t timerId) {

    vTimer_t* pTimer = TimerGetById (timerId);
    if (pTimer == NULL) {
        LOG_ERROR ("Invalid timer ID: %u", (uint16_t)timerId);
        return NULL;
    }

    uint32_t channelIdx = TIMER_ID2CHANNEL_IDX (timerId);
    if (channelIdx >= TIMER_NCHANNELS) {
        LOG_ERROR ("Invalid timer channel index: %u", (uint16_t)channelIdx);
        return NULL;
    }
    return &pTimer->channels[channelIdx];
}

/*
 * DMA is NOT supported on all timers
 */
static uint32_t TimerID2DMARequestID (eTIMER_ID_t timerId) {

    uint32_t channelIdx = TIMER_ID2CHANNEL_IDX (timerId);
    switch (TIMER_ID_CLEAR_CHANNEL_BITS (timerId)) {
    case eTIMER_5_DEV_ID: return DMA_REQUEST_TIM5_CH1 + channelIdx;
    case eTIMER_8_DEV_ID: return DMA_REQUEST_TIM8_CH1 + channelIdx;
    default:
        LOG_ERROR ("Invalid timer ID for DMA request: %u", (uint16_t)timerId);
        return 0;
    }
}

static eSTATUS_t
TimerClockInit (vTimer_t* pTimer, TimerChannel_t* pChannel, TimerInitConf_t conf) {

    if (pTimer == NULL || pChannel == NULL) {
        LOG_ERROR ("Invalid timer pointer");
        return eSTATUS_FAILURE;
    }

    eTIMER_ID_t timerId = pChannel->timerId;
    switch (TIMER_ID_CLEAR_CHANNEL_BITS (timerId)) {
    case eTIMER_5_DEV_ID: __HAL_RCC_TIM5_CLK_ENABLE (); break;
    case eTIMER_8_DEV_ID: __HAL_RCC_TIM8_CLK_ENABLE (); break;
    case eTIMER_12_DEV_ID: __HAL_RCC_TIM12_CLK_ENABLE (); break;
    case eTIMER_13_DEV_ID: __HAL_RCC_TIM13_CLK_ENABLE (); break;
    default:
        LOG_ERROR ("Invalid timer ID: %u", (uint16_t)timerId);
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

static eSTATUS_t
TimerPWMInit (vTimer_t* pTimer, TimerChannel_t* pChannel, TimerInitConf_t conf) {

    if (pTimer == NULL || pChannel == NULL) {
        LOG_ERROR ("Invalid timer pointer");
        return eSTATUS_FAILURE;
    }

    eTIMER_ID_t timerId = pChannel->timerId;
    uint32_t channel    = TIMER_ID2HALCHANNEL (timerId);
    uint32_t autoReload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    uint32_t prescaler  = (uint16_t)(SystemCoreClock / 1000000U) - 1U;
    uint32_t period = (uint16_t)(1000000U / MAX_U32 (conf.hzPeriod, 1U)) - 1U;

    if (pTimer->isTimerInitialized == false) {

        if (conf.doAutoPreload == false) {
            LOG_INFO ("Disabling auto-reload for PWM timer");
            autoReload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        }

        pTimer->handle.Instance           = TimerGetInstanceById (timerId);
        pTimer->handle.Init.Prescaler     = prescaler;
        pTimer->handle.Init.Period        = period;
        pTimer->handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        pTimer->handle.Init.CounterMode   = TIM_COUNTERMODE_UP;
        pTimer->handle.Init.RepetitionCounter = 0;
        pTimer->handle.Init.AutoReloadPreload = autoReload;

        if (HAL_TIM_PWM_Init (&pTimer->handle) != HAL_OK) {
            LOG_ERROR ("Failed to initialize timer PWM");
            return eSTATUS_FAILURE;
        }
    }

    TIM_OC_InitTypeDef sConfig = { 0 };
    sConfig.OCMode             = TIM_OCMODE_PWM1;
    sConfig.OCPolarity         = TIM_OCPOLARITY_HIGH;
    sConfig.Pulse              = 0;
    sConfig.OCNPolarity        = TIM_OCNPOLARITY_HIGH;
    sConfig.OCFastMode         = TIM_OCFAST_DISABLE;
    sConfig.OCIdleState        = TIM_OCIDLESTATE_RESET;
    sConfig.OCNIdleState       = TIM_OCNIDLESTATE_RESET;

    if (HAL_TIM_PWM_ConfigChannel (&pTimer->handle, &sConfig, channel) != HAL_OK) {
        LOG_ERROR ("Failed to configure PWM channel");
        return eSTATUS_FAILURE;
    }

    if (conf.usingDMA == true) {
        if (TimerDMAInit (pTimer, pChannel, conf) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize Timer DMA");
            return eSTATUS_FAILURE;
        }
    }

    return eSTATUS_SUCCESS;
}

static eSTATUS_t
TimerDMAInit (vTimer_t* pTimer, TimerChannel_t* pChannel, TimerInitConf_t conf) {

    if (pTimer == NULL || pChannel == NULL) {
        LOG_ERROR ("Invalid timer pointer");
        return eSTATUS_FAILURE;
    }

    eSTATUS_t status    = eSTATUS_SUCCESS;
    eTIMER_ID_t timerId = pChannel->timerId;
    bool usingDMA       = pChannel->usingDMA;
    if (usingDMA == false) {
        return eSTATUS_FAILURE;
    }
    eDMA_STREAM_ID_t dmaStreamId = eDMA_STREAM_MAX;
    uint32_t dmaRequestId        = TimerID2DMARequestID (timerId);
    DMA_INIT_TIMER_PWM (&status, dmaRequestId, &dmaStreamId);

    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize DMA");
        return eSTATUS_FAILURE;
    }

    DMAStream_t* pDMA = DMAGetStreamById (dmaStreamId);
    if (pDMA == NULL) {
        LOG_ERROR ("Failed to get DMA stream by ID");
        return eSTATUS_FAILURE;
    }

    /*
     * Link the DMA handle to the specific capture/compare timer
     * register it is going to be writing to.
     */
    uint32_t channelIdx = TIMER_ID2CHANNEL_IDX (timerId);
    switch (channelIdx) {
    case 0: pTimer->handle.hdma[TIM_DMA_ID_CC1] = &pDMA->handle; break;
    case 1: pTimer->handle.hdma[TIM_DMA_ID_CC2] = &pDMA->handle; break;
    case 2: pTimer->handle.hdma[TIM_DMA_ID_CC3] = &pDMA->handle; break;
    case 3: pTimer->handle.hdma[TIM_DMA_ID_CC4] = &pDMA->handle; break;
    default:
        LOG_ERROR ("Invalid timer channel index: %u", channelIdx);
        return eSTATUS_FAILURE;
    }
    // Link the DMA handle to the parent timer
    pDMA->handle.Parent = (TIM_HandleTypeDef*)&pTimer->handle;
    return eSTATUS_SUCCESS;
}

eSTATUS_t TimerInitGPIO (vTimer_t* pTimer, TimerChannel_t* pChannel, TimerInitConf_t conf) {

    if (pTimer == NULL || pChannel == NULL) {
        LOG_ERROR ("Invalid timer pointer");
        return eSTATUS_FAILURE;
    }

    eSTATUS_t status                = eSTATUS_SUCCESS;
    TimerBoardConf_t boardConf      = conf.timerBoardConf;
    eTIMER_ID_t timerId             = boardConf.timerId;
    GPIOBoardConf_t* pGPIOBoardConf = boardConf.pGPIOBoardConf;
    GPIO_INIT (&status, timerId, *pGPIOBoardConf);
    RETURN_IF (STATUS_FAIL (status), status, "Failed to initialize timer GPIO");

    return eSTATUS_SUCCESS;
}

eSTATUS_t TimerInit (TimerInitConf_t conf, Timer_t* pOutTimer, TimerChannel_t* pOutChannel) {

    TimerBoardConf_t boardConf = conf.timerBoardConf;
    eTIMER_ID_t timerId        = boardConf.timerId;
    eDEVICE_ID_t deviceId      = conf.deviceId;
    eTIMER_MODE_t mode         = conf.mode;
    uint32_t hzPeriod          = conf.hzPeriod;
    (void)hzPeriod;
    bool usingDMA      = conf.usingDMA;
    bool doAutoPreload = conf.doAutoPreload;
    (void)doAutoPreload;

    vTimer_t* pTimer = TimerGetById (timerId);
    if (pOutTimer != NULL) {
        pTimer = (vTimer_t*)pOutTimer;
    }
    TimerChannel_t* pChannel = TimerGetChannelById (timerId);
    if (pOutChannel != NULL) {
        pChannel = (TimerChannel_t*)pOutChannel;
    }
    if (pTimer == NULL || pChannel == NULL) {
        LOG_ERROR ("Failed to find timer or timer handle");
        return eSTATUS_FAILURE;
    }

    if (pTimer->isTimerInitialized == false) {

        memset (pTimer, 0, sizeof (vTimer_t));
        pChannel->timerId  = timerId;
        pChannel->deviceId = deviceId;
        pChannel->mode     = mode;
        pChannel->usingDMA = usingDMA;
        if (TimerClockInit (pTimer, pChannel, conf) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize timer clock");
            goto error;
        }
    }

    if (pChannel->isChannelInitialized == true) {
        LOG_ERROR ("Timer channel already initialized");
        return eSTATUS_FAILURE;
    }

    if (conf.mode == eTIMER_MODE_PWM) {

        if (TimerPWMInit (pTimer, pChannel, conf) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize timer PWM");
            goto error;
        }

    } else {
        LOG_ERROR ("Unsupported timer mode: %u", (uint16_t)conf.mode);
        goto error;
    }

    if (TimerInitGPIO (pTimer, pChannel, conf) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize timer GPIO");
        goto error;
    }

    pChannel->isChannelInitialized = true;
    pTimer->isTimerInitialized     = true;
    return eSTATUS_SUCCESS;

error:
    memset (pTimer, 0, sizeof (vTimer_t));
    return eSTATUS_FAILURE;
}

eSTATUS_t
TimerStart (Timer_t* pTimer, TimerChannel_t* pChannel, uint32_t const* pData, uint16_t Length) {

    if (pTimer == NULL || pChannel == NULL) {
        LOG_ERROR ("Failed to get timer or channel by ID");
        return eSTATUS_FAILURE;
    }

    TIM_HandleTypeDef* pHandle = &pTimer->handle;
    eTIMER_MODE_t mode         = pChannel->mode;
    bool usingDMA              = pChannel->usingDMA;
    uint32_t channel           = TIMER_ID2HALCHANNEL (pChannel->timerId);

    if (usingDMA == true && mode == eTIMER_MODE_PWM) {

        if (pData == NULL || Length == 0) {
            LOG_ERROR ("Invalid data or length for PWM DMA start");
            return eSTATUS_FAILURE;
        }

        if (HAL_TIM_PWM_Start_DMA (pHandle, channel, (uint32_t*)pData, Length) != HAL_OK) {
            LOG_ERROR ("Failed to start PWM DMA");
            return eSTATUS_FAILURE;
        }

    } else if (usingDMA == false && mode == eTIMER_MODE_PWM) {

        if (HAL_TIM_PWM_Start (pHandle, channel) != HAL_OK) {
            LOG_ERROR ("Failed to start PWM");
            return eSTATUS_FAILURE;
        }

    } else {
        LOG_ERROR ("Unsupported timer mode or DMA usage");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t TimerStop (Timer_t* pTimer, TimerChannel_t* pChannel) {

    if (pTimer == NULL || pChannel == NULL) {
        LOG_ERROR ("Failed to get timer or channel by ID");
        return eSTATUS_FAILURE;
    }

    TIM_HandleTypeDef* pHandle = &pTimer->handle;
    eTIMER_MODE_t mode         = pChannel->mode;
    bool usingDMA              = pChannel->usingDMA;
    uint32_t channel           = TIMER_ID2HALCHANNEL (pChannel->timerId);

    if (usingDMA == true && mode == eTIMER_MODE_PWM) {

        if (HAL_TIM_PWM_Stop_DMA (pHandle, channel) != HAL_OK) {
            LOG_ERROR ("Failed to stop PWM DMA");
            return eSTATUS_FAILURE;
        }

    } else if (usingDMA == false && mode == eTIMER_MODE_PWM) {

        if (HAL_TIM_PWM_Stop (pHandle, channel) != HAL_OK) {
            LOG_ERROR ("Failed to stop PWM");
            return eSTATUS_FAILURE;
        }

    } else {
        LOG_ERROR ("Unsupported timer mode or DMA usage");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t TimerWrite (Timer_t* pTimer, TimerChannel_t* pChannel, uint32_t usUpTime) {

    if (pTimer == NULL) {
        LOG_ERROR ("Failed to get timer by ID");
        return eSTATUS_FAILURE;
    }
    /* ARR is synonymous with microseconds because the clock is prescaled to 1MHz */
    /* Set ARR and PSC here because sometimes they don't get set properly */
    pTimer->handle.Instance->ARR = pTimer->handle.Init.Period;
    pTimer->handle.Instance->PSC = pTimer->handle.Init.Prescaler;

    TIMER_SET_COMPARE (pChannel->timerId, usUpTime);

    return eSTATUS_SUCCESS;
}

eSTATUS_t
TimerRegisterCallback (Timer_t* pTimer, TimerChannel_t* pChannel, TimerCallback_t callback, eTIMER_CALLBACK_ID_t cbType) {

    if (callback == NULL) {
        LOG_ERROR ("Invalid parameters for Timer callback registration");
        return eSTATUS_FAILURE;
    }

    if (pChannel == NULL) {
        LOG_ERROR ("Failed to get timer channel by ID");
        return eSTATUS_FAILURE;
    }

    switch (cbType) {
    case eTIMER_CALLBACK_TRANSFER_COMPLETE:
        pChannel->pulseFinishedCallback = callback;
        break;
    case eTIMER_CALLBACK_TRANSFER_ERROR:
        pChannel->errorCallback = callback;
        break;
    default:
        LOG_ERROR ("Unsupported callback type");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t
TimerSetRegister (Timer_t* pTimer, TimerChannel_t* pChannel, eTIMER_SET_REG_t regType, uint32_t value) {

    if (pTimer == NULL) {
        LOG_ERROR ("Failed to get timer by ID");
        return eSTATUS_FAILURE;
    }

    switch (regType) {
    case eTIMER_SET_REG_PRESCALER:
        __HAL_TIM_SET_PRESCALER (&pTimer->handle, value);
        break;
    case eTIMER_SET_REG_PERIOD:
        __HAL_TIM_SET_AUTORELOAD (&pTimer->handle, value);
        break;
    case eTIMER_SET_REG_COMPARE:
        __HAL_TIM_SET_COMPARE (
        &pTimer->handle,
        TIMER_ID2HALCHANNEL (pChannel->timerId),
        value
        );
        break;
    default:
        LOG_ERROR ("Unsupported register type");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eTIMER_CHANNEL_STATE_t TimerGetChannelState (Timer_t* pTimer, TimerChannel_t* pChannel) {

    if (pTimer == NULL) {
        LOG_ERROR ("Failed to get timer by ID");
        return eSTATUS_FAILURE;
    }

    HAL_TIM_ChannelStateTypeDef channelState =
    TIM_CHANNEL_STATE_GET (&pTimer->handle, TIMER_ID2HALCHANNEL (pChannel->timerId));

    switch (channelState) {
    case HAL_TIM_CHANNEL_STATE_RESET: return eTIMER_CHANNEL_STATE_RESET;
    case HAL_TIM_CHANNEL_STATE_READY: return eTIMER_CHANNEL_STATE_READY;
    case HAL_TIM_CHANNEL_STATE_BUSY: return eTIMER_CHANNEL_STATE_BUSY;
    }

    return eTIMER_CHANNEL_STATE_ERROR;
}