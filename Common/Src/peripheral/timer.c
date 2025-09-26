#include "peripheral/timer.h"
#include "common.h"
#include "conf/conf.h"
#include "dma.h"
#include "hal.h"
#include "log/logger.h"
#include "mem/mem.h"
#include "peripheral/gpio.h"
#include <stdint.h>
#include <string.h>

static SHARED_MEM_SECTION vTimer_t gTimers[TIMER_NTIMERS] = { 0 };

static vTimer_t* TimerGetByInstance (TIM_TypeDef* instance);
static void TimerHandleCallback (TIM_HandleTypeDef* htim, eTIMER_CALLBACK_ID_t callbackType);

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

    switch (callbackType) {
    case eTIMER_CALLBACK_TRANSFER_COMPLETE:
        if (pChannel->pulseFinishedCallback != NULL) {
            pChannel->pulseFinishedCallback (pChannel->conf.timerId);
        }
        break;
    case eTIMER_CALLBACK_TRANSFER_ERROR:
        if (pChannel->errorCallback != NULL) {
            pChannel->errorCallback (pChannel->conf.timerId);
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

static vTimer_t* TimerGetById (eTIMER_ID_t timerId) {

    uint32_t timerIdx = TIMER_ID2IDX (timerId);
    if (timerIdx >= TIMER_NTIMERS) {
        LOG_ERROR ("Invalid timer index: %u", (uint16_t)timerIdx);
        return NULL;
    }
    return &gTimers[timerIdx];
}

static TimerChannel_t* TimerGetChannelById (eTIMER_ID_t timerId) {

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

static eSTATUS_t TimerClockInit (TimerInitConf_t conf, vTimer_t* pOutTimer) {

    if (pOutTimer == NULL) {
        LOG_ERROR ("Invalid timer pointer");
        return eSTATUS_FAILURE;
    }

    switch (TIMER_ID_CLEAR_CHANNEL_BITS (conf.timerId)) {
    case eTIMER_5_DEV_ID: __HAL_RCC_TIM5_CLK_ENABLE (); break;
    case eTIMER_8_DEV_ID: __HAL_RCC_TIM8_CLK_ENABLE (); break;
    case eTIMER_12_DEV_ID: __HAL_RCC_TIM12_CLK_ENABLE (); break;
    case eTIMER_13_DEV_ID: __HAL_RCC_TIM13_CLK_ENABLE (); break;
    default:
        LOG_ERROR ("Invalid timer ID: %u", (uint16_t)conf.timerId);
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

static eSTATUS_t TimerPWMInit (TimerInitConf_t conf, vTimer_t* pOutTimer) {

    if (pOutTimer == NULL) {
        LOG_ERROR ("Invalid timer pointer");
        return eSTATUS_FAILURE;
    }

    uint32_t channel    = TIMER_ID2HALCHANNEL (conf.timerId);
    uint32_t autoReload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    uint32_t prescaler  = (uint16_t)(SystemCoreClock / 1000000U) - 1U;
    uint32_t period = (uint16_t)(1000000U / MAX_U32 (conf.hzPeriod, 1U)) - 1U;

    if (pOutTimer->isTimerInitialized == FALSE) {

        if (conf.doAutoPreload == FALSE) {
            LOG_INFO ("Disabling auto-reload for PWM timer");
            autoReload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        }

        pOutTimer->handle.Instance = TimerGetInstanceById (conf.timerId);
        pOutTimer->handle.Init.Prescaler         = prescaler;
        pOutTimer->handle.Init.Period            = period;
        pOutTimer->handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
        pOutTimer->handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
        pOutTimer->handle.Init.RepetitionCounter = 0;
        pOutTimer->handle.Init.AutoReloadPreload = autoReload;

        if (HAL_TIM_PWM_Init (&pOutTimer->handle) != HAL_OK) {
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

    if (HAL_TIM_PWM_ConfigChannel (&pOutTimer->handle, &sConfig, channel) != HAL_OK) {
        LOG_ERROR ("Failed to configure PWM channel");
        return eSTATUS_FAILURE;
    }

    if (conf.usingDMA == TRUE) {
        if (TimerDMAInit (conf, pOutTimer) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize Timer DMA");
            return eSTATUS_FAILURE;
        }
    }

    return eSTATUS_SUCCESS;
}

static eSTATUS_t TimerDMAInit (TimerInitConf_t conf, vTimer_t* pOutTimer) {

    if (pOutTimer == NULL || conf.usingDMA == FALSE) {
        LOG_ERROR ("Invalid timer pointer");
        return eSTATUS_FAILURE;
    }

    eDMA_STREAM_ID_t dmaStreamId = eDMA_STREAM_MAX;
    DMAInitConf_t dmaConfig      = { 0 };
    dmaConfig.direction          = eDMA_DIRECTION_MEMORY_TO_PERIPH;
    dmaConfig.priority           = eDMA_PRIORITY_HIGH;
    dmaConfig.request            = TimerID2DMARequestID (conf.timerId);
    dmaConfig.transferMode       = DMA_NORMAL;
    dmaConfig.fifoMode           = DMA_FIFOMODE_DISABLE;

    if (DMAInit (dmaConfig, &dmaStreamId) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize DMA");
        return eSTATUS_FAILURE;
    }

    DMAStream_t* pDMA = DMAGetStreamById (dmaStreamId);
    if (pDMA == NULL) {
        LOG_ERROR ("Failed to get DMA stream by ID");
        return eSTATUS_FAILURE;
    }

    /*
     * Link the DMA handle to the specific capture/compare timer register
     * it is going to be writing to.
     */
    uint32_t channelIdx = TIMER_ID2CHANNEL_IDX (conf.timerId);
    switch (channelIdx) {
    case 0: pOutTimer->handle.hdma[TIM_DMA_ID_CC1] = &pDMA->handle; break;
    case 1: pOutTimer->handle.hdma[TIM_DMA_ID_CC2] = &pDMA->handle; break;
    case 2: pOutTimer->handle.hdma[TIM_DMA_ID_CC3] = &pDMA->handle; break;
    case 3: pOutTimer->handle.hdma[TIM_DMA_ID_CC4] = &pDMA->handle; break;
    default:
        LOG_ERROR ("Invalid timer channel index: %u", channelIdx);
        return eSTATUS_FAILURE;
    }
    // Link the DMA handle to the parent timer
    pDMA->handle.Parent = &pOutTimer->handle;
    return eSTATUS_SUCCESS;
}

eSTATUS_t TimerInitGPIO (TimerChannel_t* pTimerChannel) {

    if (pTimerChannel == NULL) {
        LOG_ERROR ("Invalid timer pointer");
        return eSTATUS_FAILURE;
    }

    eTIMER_ID_t timerId         = pTimerChannel->conf.timerId;
    eTIMER_DEV_ID_t deviceId    = TIMER_ID_CLEAR_CHANNEL_BITS (timerId);
    eTIMER_CHANNEL_ID_t channel = TIMER_ID2CHANNEL_IDX (timerId);
    eGPIO_ID_t gpioId           = eGPIO_ID_NULL;
    uint32_t alternate          = 0;

    if (deviceId == eTIMER_5_DEV_ID) {
        alternate = GPIO_AF2_TIM5;
        if (channel == eTIMER_CHANNEL_1_ID) {
            gpioId = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_0); // TIM5_CH1
        } else if (channel == eTIMER_CHANNEL_2_ID) {
            gpioId = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_1); // TIM5_CH2
        } else if (channel == eTIMER_CHANNEL_3_ID) {
            gpioId = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_2); // TIM5_CH3
        } else if (channel == eTIMER_CHANNEL_4_ID) {
            gpioId = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_3); // TIM5_CH4
        }
    } else if (deviceId == eTIMER_8_DEV_ID) {
        alternate = GPIO_AF3_TIM8;
        if (channel == eTIMER_CHANNEL_1_ID) {
            gpioId = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_6); // TIM8_CH1
        } else if (channel == eTIMER_CHANNEL_2_ID) {
            gpioId = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_7); // TIM8_CH2
        } else if (channel == eTIMER_CHANNEL_3_ID) {
            gpioId = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_8); // TIM8_CH3
        } else if (channel == eTIMER_CHANNEL_4_ID) {
            gpioId = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_9); // TIM8_CH4
        }
    } else if (deviceId == eTIMER_12_DEV_ID) {
        alternate = GPIO_AF2_TIM12;
        if (channel == eTIMER_CHANNEL_1_ID) {
            gpioId = GPIO_ID_MAKE (eGPIO_PORTID_H, eGPIO_PINID_6); // TIM12_CH1
        } else if (channel == eTIMER_CHANNEL_2_ID) {
            gpioId = GPIO_ID_MAKE (eGPIO_PORTID_H, eGPIO_PINID_9); // TIM12_CH2
        }
    } // else if (deviceId == eTIMER_13_DEV_ID) {
      // if (channel == eTIMER_CHANNEL_1_ID) {
      //  gpioId = GPIO_ID_MAKE (eGPIO_PORTID_F, eGPIO_PINID_8); // TIM13_CH1
    //}
    eSTATUS_t status = eSTATUS_SUCCESS;
    GPIO_INIT_TIMER (&status, timerId, gpioId, alternate);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize GPIO for timer");
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t TimerInit (TimerInitConf_t conf) {

    vTimer_t* pTimer = TimerGetById (conf.timerId);
    if (pTimer == NULL) {
        LOG_ERROR ("Failed to find timer or timer handle");
        return eSTATUS_FAILURE;
    }

    if (pTimer->isTimerInitialized == FALSE) {

        memset (pTimer, 0, sizeof (vTimer_t));
        if (TimerClockInit (conf, pTimer) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize timer clock");
            goto error;
        }
    }

    TimerChannel_t* pChannel = TimerGetChannelById (conf.timerId);
    if (pChannel == NULL) {
        LOG_ERROR ("Timer channel was null or already initialized");
        return eSTATUS_FAILURE;
    }

    if (pChannel->isChannelInitialized == TRUE) {
        LOG_ERROR ("Timer channel already initialized");
        return eSTATUS_FAILURE;
    }

    memset (pChannel, 0, sizeof (TimerChannel_t));
    pChannel->conf = conf;

    if (conf.mode == eTIMER_MODE_PWM) {

        if (TimerPWMInit (conf, pTimer) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize timer PWM");
            goto error;
        }

    } else {
        LOG_ERROR ("Unsupported timer mode: %u", (uint16_t)conf.mode);
        goto error;
    }

    if (TimerInitGPIO (pChannel) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize timer GPIO");
        goto error;
    }

    pChannel->isChannelInitialized = TRUE;
    pTimer->isTimerInitialized     = TRUE;
    return eSTATUS_SUCCESS;

error:
    memset (pTimer, 0, sizeof (vTimer_t));
    return eSTATUS_FAILURE;
}

eSTATUS_t TimerStart (eTIMER_ID_t timerId, uint32_t const* pData, uint16_t Length) {

    vTimer_t* pTimer         = TimerGetById (timerId);
    TimerChannel_t* pChannel = TimerGetChannelById (timerId);
    if (pTimer == NULL || pChannel == NULL) {
        LOG_ERROR ("Failed to get timer or channel by ID");
        return eSTATUS_FAILURE;
    }

    TIM_HandleTypeDef* pHandle = &pTimer->handle;
    TimerInitConf_t* pConf     = &pChannel->conf;
    uint32_t channel           = TIMER_ID2HALCHANNEL (timerId);

    if (pConf->usingDMA == TRUE && pConf->mode == eTIMER_MODE_PWM) {

        if (pData == NULL || Length == 0) {
            LOG_ERROR ("Invalid data or length for PWM DMA start");
            return eSTATUS_FAILURE;
        }

        if (HAL_TIM_PWM_Start_DMA (pHandle, channel, (uint32_t*)pData, Length) != HAL_OK) {
            LOG_ERROR ("Failed to start PWM DMA");
            return eSTATUS_FAILURE;
        }

    } else if (pConf->usingDMA == FALSE && pConf->mode == eTIMER_MODE_PWM) {

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

eSTATUS_t TimerStop (eTIMER_ID_t timerId) {

    vTimer_t* pTimer         = TimerGetById (timerId);
    TimerChannel_t* pChannel = TimerGetChannelById (timerId);
    if (pTimer == NULL || pChannel == NULL) {
        LOG_ERROR ("Failed to get timer or channel by ID");
        return eSTATUS_FAILURE;
    }

    TIM_HandleTypeDef* pHandle = &pTimer->handle;
    TimerInitConf_t* pConf     = &pChannel->conf;
    uint32_t channel           = TIMER_ID2HALCHANNEL (timerId);

    if (pConf->usingDMA == TRUE && pConf->mode == eTIMER_MODE_PWM) {

        if (HAL_TIM_PWM_Stop_DMA (pHandle, channel) != HAL_OK) {
            LOG_ERROR ("Failed to stop PWM DMA");
            return eSTATUS_FAILURE;
        }

    } else if (pConf->usingDMA == FALSE && pConf->mode == eTIMER_MODE_PWM) {

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

eSTATUS_t TimerWrite (eTIMER_ID_t timerId, uint32_t usUpTime) {

    vTimer_t* pTimer = TimerGetById (timerId);
    if (pTimer == NULL) {
        LOG_ERROR ("Failed to get timer by ID");
        return eSTATUS_FAILURE;
    }
    /* ARR is synonymous with microseconds because the clock is prescaled to 1MHz */
    /* Set ARR and PSC here because sometimes they don't get set properly */
    pTimer->handle.Instance->ARR = pTimer->handle.Init.Period;
    pTimer->handle.Instance->PSC = pTimer->handle.Init.Prescaler;

    TIMER_SET_COMPARE (timerId, usUpTime);

    return eSTATUS_SUCCESS;
}

eSTATUS_t TimerRegisterCallback (eTIMER_ID_t timerId, TimerCallback_t callback, eTIMER_CALLBACK_ID_t cbType) {

    if (callback == NULL) {
        LOG_ERROR ("Invalid parameters for Timer callback registration");
        return eSTATUS_FAILURE;
    }

    TimerChannel_t* pChannel = TimerGetChannelById (timerId);
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

eSTATUS_t TimerSetRegister (eTIMER_ID_t timerId, eTIMER_SET_REG_t regType, uint32_t value) {

    vTimer_t* pTimer = TimerGetById (timerId);
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
        __HAL_TIM_SET_COMPARE (&pTimer->handle, TIMER_ID2HALCHANNEL (timerId), value);
        break;
    default:
        LOG_ERROR ("Unsupported register type");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eTIMER_CHANNEL_STATE_t TimerGetChannelState (eTIMER_ID_t timerId) {

    vTimer_t* pTimer = TimerGetById (timerId);
    if (pTimer == NULL) {
        LOG_ERROR ("Failed to get timer by ID");
        return eSTATUS_FAILURE;
    }

    HAL_TIM_ChannelStateTypeDef channelState =
    TIM_CHANNEL_STATE_GET (&pTimer->handle, TIMER_ID2HALCHANNEL (timerId));

    switch (channelState) {
    case HAL_TIM_CHANNEL_STATE_RESET: return eTIMER_CHANNEL_STATE_RESET;
    case HAL_TIM_CHANNEL_STATE_READY: return eTIMER_CHANNEL_STATE_READY;
    case HAL_TIM_CHANNEL_STATE_BUSY: return eTIMER_CHANNEL_STATE_BUSY;
    }

    return eTIMER_CHANNEL_STATE_ERROR;
}