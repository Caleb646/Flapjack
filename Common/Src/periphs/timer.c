#include "periphs/timer.h"
#include "common.h"
#include "conf.h"
#include "dma.h"
#include "hal.h"
#include "log/logger.h"
#include "periphs/gpio.h"
#include <stdint.h>


#define MAX_DMA_TIM_REGISTERS 7U

// 13 timers, 4 events, and 8 timer channels
PWM_DMACallback gPWM_DMACallbacks[13][4][8] = { 0 };

static int32_t PWMTim2Idx (TIM_TypeDef* pTim);
static int32_t PWMTimChannel2Idx (uint32_t channeldID);
static int32_t PWMTimActiveChannel2Idx (HAL_TIM_ActiveChannel channel);
static PWM_DMACallback
PWM_DMAGetCallback (TIM_HandleTypeDef* htim, ePWM_DMA_CB_TYPE cbType);

/*
 * The callback set by HAL_PWM_DMA_START call this function for Timer DMA transfer error callback.
 */
void HAL_TIM_ErrorCallback (TIM_HandleTypeDef* htim) {

    if (htim == NULL) {
        return;
    }
    PWM_DMACallback callback = PWM_DMAGetCallback (htim, ePWM_DMA_CB_TRANSFER_ERROR);
    if (callback != NULL) {
        callback (htim);
    }
}

/*
 * The callback set by HAL_PWM_DMA_START call this function for Timer DMA transfer half complete callback.
 */
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback (TIM_HandleTypeDef* htim) {

    if (htim == NULL) {
        return;
    }
    PWM_DMACallback callback = PWM_DMAGetCallback (htim, ePWM_DMA_CB_HALF_TRANSFER);
    if (callback != NULL) {
        callback (htim);
    }
}

/*
 * The callback set by HAL_PWM_DMA_START call this function for Timer DMA transfer complete callback.
 */
void HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef* htim) {

    if (htim == NULL) {
        return;
    }
    PWM_DMACallback callback = PWM_DMAGetCallback (htim, ePWM_DMA_CB_TRANSFER_COMPLETE);
    if (callback != NULL) {
        callback (htim);
    }
}

static PWM_DMACallback
PWM_DMAGetCallback (TIM_HandleTypeDef* htim, ePWM_DMA_CB_TYPE cbType) {

    int32_t timIdx     = PWMTim2Idx (htim->Instance);
    int32_t channelIdx = PWMTimActiveChannel2Idx (htim->Channel);
    if (timIdx < 0 || channelIdx < 0) {
        return NULL;
    }
    return gPWM_DMACallbacks[timIdx][cbType][channelIdx];
}

static int32_t PWMTim2Idx (TIM_TypeDef* pTim) {

    if (pTim == NULL) {
        // LOG_ERROR ("Invalid timer instance");
        return -1;
    }

    TIM_TypeDef* pTimerRegisters = pTim;
    if (pTimerRegisters == TIM8) {
        return 8 - 1; // Timers are 1-indexed in the HAL, so TIM8 is index 7
    }
    if (pTimerRegisters == TIM13) {
        return 13 - 1; // Timers are 1-indexed in the HAL, so TIM13 is index 12
    }
    return -1;
}

static int32_t PWMTimChannel2Idx (uint32_t channeldID) {

    switch (channeldID) {
    case TIM_CHANNEL_1: return 0;
    case TIM_CHANNEL_2: return 1;
    case TIM_CHANNEL_3: return 2;
    case TIM_CHANNEL_4: return 3;
    case TIM_CHANNEL_5: return 4;
    case TIM_CHANNEL_6: return 5;
    default:
        // LOG_ERROR ("Invalid timer channel ID: 0x%X", (uint16_t)channeldID);
        return -1; // Invalid channel ID
    }
}

static int32_t PWMTimActiveChannel2Idx (HAL_TIM_ActiveChannel channel) {

    if (channel > 0) {
        return (int32_t)channel - 1; // Convert to 0-indexed
    }
    return -1;
}

static Timer_t gTimers[TIMER_MAX_NUMBER_OF_HANDLES] = { 0 };

static TIM_TypeDef* TimerGetInstanceById (eTIMER_ID_t timerId) {

    // Clear channel bits
    switch (timerId & ~TIMER_ID_CHANNEL_MASK) {
    case eTIMER_5_CH1_ID: return TIM5;
    case eTIMER_8_CH1_ID: return TIM8;
    case eTIMER_12_CH1_ID: return TIM12;
    case eTIMER_13_CH1_ID: return TIM13;
    default:
        LOG_ERROR ("Invalid timer ID: %u", (uint16_t)timerId);
        return NULL;
    }
}

static Timer_t* TimerGetById (eTIMER_ID_t timerId) {

    uint32_t timerIdx = (timerId & ~TIMER_ID_CHANNEL_MASK) >> TIMER_ID_CHANNEL_MASK;
    if (timerIdx >= (sizeof (gTimers) / sizeof (gTimers[0]))) {
        LOG_ERROR ("Invalid timer index: %u", (uint16_t)timerIdx);
        return NULL;
    }
    return &gTimers[timerIdx];
}

static TimerChannel_t* TimerGetChannelById (eTIMER_ID_t timerId) {

    Timer_t* pTimer = TimerGetById (timerId);
    if (pTimer == NULL) {
        LOG_ERROR ("Invalid timer ID: %u", (uint16_t)timerId);
        return NULL;
    }
    uint32_t channelIdx = timerId & TIMER_ID_CHANNEL_MASK;
    if (channelIdx >= TIMER_MAX_NUMBER_OF_CHANNELS) {
        LOG_ERROR ("Invalid timer channel index: %u", (uint16_t)channelIdx);
        return NULL;
    }
    return &pTimer->channels[channelIdx];
}

static uint32_t TimerID2DMARequestID (eTIMER_ID_t timerId) {

    switch (timerId) {

    case eTIMER_5_CH1_ID: return DMA_REQUEST_TIM5_CH1;
    case eTIMER_5_CH2_ID: return DMA_REQUEST_TIM5_CH2;
    case eTIMER_5_CH3_ID: return DMA_REQUEST_TIM5_CH3;
    case eTIMER_5_CH4_ID: return DMA_REQUEST_TIM5_CH4;

    case eTIMER_8_CH1_ID: return DMA_REQUEST_TIM8_CH1;
    case eTIMER_8_CH2_ID: return DMA_REQUEST_TIM8_CH2;
    case eTIMER_8_CH3_ID: return DMA_REQUEST_TIM8_CH3;
    case eTIMER_8_CH4_ID: return DMA_REQUEST_TIM8_CH4;

    default:
        LOG_ERROR ("Invalid timer ID for DMA request: %u", (uint16_t)timerId);
        return 0;
    }
}

static eSTATUS_t TimerClockInit (TimerInitConf_t const* pConf, Timer_t* pOutTimer) {

    if (pOutTimer == NULL) {
        LOG_ERROR ("Invalid timer pointer");
        return eSTATUS_FAILURE;
    }
    uint32_t timerId = pConf->timerId & ~TIMER_ID_CHANNEL_MASK;
    if (timerId == eTIMER_5_CH1_ID) {
        __HAL_RCC_TIM5_CLK_ENABLE ();
    } else if (timerId == eTIMER_8_CH1_ID) {
        __HAL_RCC_TIM8_CLK_ENABLE ();
    } else if (timerId == eTIMER_12_CH1_ID) {
        __HAL_RCC_TIM12_CLK_ENABLE ();
    } else if (timerId == eTIMER_13_CH1_ID) {
        __HAL_RCC_TIM13_CLK_ENABLE ();
    } else {
        LOG_ERROR ("Invalid timer ID: %u", (uint16_t)timerId);
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

static eSTATUS_t TimerPWMInit (TimerInitConf_t const* pConf, Timer_t* pOutTimer) {

    if (pOutTimer == NULL) {
        LOG_ERROR ("Invalid timer pointer");
        return eSTATUS_FAILURE;
    }

    uint32_t channel    = TIMER_ID2CHANNEL (pConf->timerId);
    uint32_t autoReload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    uint32_t prescaler  = (uint16_t)(SystemCoreClock / 1000000U) - 1U;
    uint32_t period = (uint16_t)(1000000U / MAX_U32 (pConf->hzPeriod, 1U)) - 1U;

    if (pOutTimer->isTimerInitialized == FALSE) {

        if (pConf->doAutoPreload == FALSE) {
            LOG_INFO ("Enabling auto-reload for PWM timer");
            autoReload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        }

        pOutTimer->handle.Instance = TimerGetInstanceById (pConf->timerId);
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

    if (GPIOInitTimer (pOutTimer->handle.Instance, channel, NULL) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize GPIO for timer");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

static eSTATUS_t TimerDMAInit (TimerInitConf_t const* pConf, Timer_t* pOutTimer) {

    if (pOutTimer == NULL || pConf->usingDMA == FALSE) {
        LOG_ERROR ("Invalid timer pointer");
        return eSTATUS_FAILURE;
    }

    eDMA_STREAM_ID_t dmaStreamID = eDMA_STREAM_MAX;
    DMAInitConf_t dmaConfig      = { 0 };
    dmaConfig.direction          = eDMA_DIRECTION_MEMORY_TO_PERIPH;
    dmaConfig.priority           = eDMA_PRIORITY_HIGH;
    dmaConfig.request            = TimerID2DMARequestID (pConf->timerId);
    dmaConfig.transferMode       = DMA_NORMAL;
    dmaConfig.fifoMode           = DMA_FIFOMODE_DISABLE;

    if (DMAInit (dmaConfig, &dmaStreamID) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize DMA");
        return eSTATUS_FAILURE;
    }

    DMAStream_t* pDMA = DMAGetStreamById (dmaStreamID);
    if (pDMA == NULL) {
        LOG_ERROR ("Failed to get DMA stream by ID");
        return eSTATUS_FAILURE;
    }

    /*
     * Link the DMA handle to the specific PWM timer register it is
     * going to be writing to.
     */
    uint32_t channelIdx = pConf->timerId & TIMER_ID_CHANNEL_MASK;
    switch (channelIdx) {
    case 0: pOutTimer->handle.hdma[TIM_DMA_ID_CC1] = pDMA; break;
    case 1: pOutTimer->handle.hdma[TIM_DMA_ID_CC2] = pDMA; break;
    case 2: pOutTimer->handle.hdma[TIM_DMA_ID_CC3] = pDMA; break;
    case 3: pOutTimer->handle.hdma[TIM_DMA_ID_CC4] = pDMA; break;
    default:
        LOG_ERROR ("Invalid timer channel index: %u", channelIdx);
        return eSTATUS_FAILURE;
    }
    // Link the DMA handle to the parent timer
    pDMA->handle.Parent = &pOutTimer->handle;
    return eSTATUS_SUCCESS;
}

eSTATUS_t TimerInit (TimerInitConf_t const* pConf) {

    if (pConf == NULL) {
        LOG_ERROR ("Invalid timer initialization configuration");
        return eSTATUS_FAILURE;
    }

    Timer_t* pTimer = TimerGetById (pConf->timerId);
    if (pTimer == NULL) {
        LOG_ERROR ("Failed to find timer or timer handle");
        return eSTATUS_FAILURE;
    }

    if (pTimer->isTimerInitialized == FALSE) {

        memset (pTimer, 0, sizeof (Timer_t));
        if (TimerClockInit (pConf, pTimer) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize timer clock");
            goto error;
        }
    }

    TimerChannel_t* pChannel = TimerGetChannelById (pConf->timerId);
    if (pChannel == NULL || pChannel->isChannelInitialized == TRUE) {
        LOG_ERROR ("Timer channel was null or already initialized");
        return eSTATUS_FAILURE;
    }

    memset (pChannel, 0, sizeof (TimerChannel_t));
    pChannel->deviceId = pConf->deviceId;
    pChannel->timerId  = pConf->timerId;
    pChannel->mode     = pConf->mode;
    pChannel->usingDMA = pConf->usingDMA;

    if (pConf->mode == eTIMER_MODE_PWM) {
        if (TimerPWMInit (pConf, pTimer) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize timer PWM");
            goto error;
        }
    } else {
        LOG_ERROR ("Unsupported timer mode: %u", (uint16_t)pConf->mode);
        goto error;
    }

    pChannel->isChannelInitialized = TRUE;
    pTimer->isTimerInitialized     = TRUE;
    return eSTATUS_SUCCESS;

error:
    memset (pTimer, 0, sizeof (Timer_t));
    return eSTATUS_FAILURE;
}

// eSTATUS_t PWMInit (PWMConfig config, PWMHandle* pOutHandle) {

//     if (pOutHandle == NULL) {
//         LOG_ERROR ("PWM output handle pointer is NULL");
//         return eSTATUS_FAILURE;
//     }

//     if (PWM_CHECK_CONF_OK (&config) == FALSE) {
//         LOG_ERROR ("Received invalid PWM configuration pointer or
//         timer pointer"); return eSTATUS_FAILURE;
//     }

//     memset (pOutHandle, 0, sizeof (PWMHandle));
//     uint32_t channelID  = config.base.channelID;
//     uint32_t autoReload = TIM_AUTORELOAD_PRELOAD_ENABLE;
//     uint32_t prescaler  = (uint16_t)(SystemCoreClock / 1000000U) - 1U;
//     uint32_t period = (uint16_t)(1000000U / MAX_U32 (config.base.hzPeriod, 1U)) - 1U;

//     if (config.base.doAutoReload == FALSE) {
//         LOG_INFO ("Enabling auto-reload for PWM timer");
//         autoReload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//     }

//     pOutHandle->timer.Instance               = config.base.pTimer;
//     pOutHandle->channelID                    = channelID;
//     pOutHandle->timer.Init.Prescaler         = prescaler;
//     pOutHandle->timer.Init.Period            = period;
//     pOutHandle->timer.Init.ClockDivision     =
//     TIM_CLOCKDIVISION_DIV1; pOutHandle->timer.Init.CounterMode =
//     TIM_COUNTERMODE_UP; pOutHandle->timer.Init.RepetitionCounter =
//     0; pOutHandle->timer.Init.AutoReloadPreload = autoReload;

//     eSTATUS_t status = eSTATUS_SUCCESS;
//     status           = PWMEnableTimClock (pOutHandle);
//     if (status != eSTATUS_SUCCESS) {
//         LOG_ERROR ("Failed to enable timer clock");
//         return status;
//     }

//     if (HAL_TIM_PWM_Init (&pOutHandle->timer) != HAL_OK) {
//         LOG_ERROR ("Failed to initialize timer PWM");
//         return eSTATUS_FAILURE;
//     }

//     TIM_OC_InitTypeDef sConfig = { 0 };
//     sConfig.OCMode             = TIM_OCMODE_PWM1;
//     sConfig.OCPolarity         = TIM_OCPOLARITY_HIGH;
//     sConfig.Pulse              = 0;
//     sConfig.OCNPolarity        = TIM_OCNPOLARITY_HIGH;
//     sConfig.OCFastMode         = TIM_OCFAST_DISABLE;
//     sConfig.OCIdleState        = TIM_OCIDLESTATE_RESET;
//     sConfig.OCNIdleState       = TIM_OCNIDLESTATE_RESET;
//     if (HAL_TIM_PWM_ConfigChannel (&pOutHandle->timer, &sConfig, channelID) != HAL_OK) {
//         LOG_ERROR ("Failed to configure PWM channel");
//         return eSTATUS_FAILURE;
//     }

//     status = PWMInitGPIO (pOutHandle);
//     if (status != eSTATUS_SUCCESS) {
//         LOG_ERROR ("Failed to initialize GPIO for timer");
//         return status;
//     }

//     return eSTATUS_SUCCESS;
// }

eSTATUS_t PWMStart (PWMHandle* pHandle) {
    if (PWM_CHECK_OK (pHandle) == FALSE) {
        LOG_ERROR ("Received invalid PWM pointer");
        return eSTATUS_FAILURE;
    }

    /* 0x1FU = 31 bits max shift */
    uint32_t tmp = TIM_CCER_CC1E << (pHandle->channelID & 0x1FU);
    /* Reset the CCxE Bit */
    pHandle->timer.Instance->CCER &= ~tmp;
    /* Set or reset the CCxE Bit */
    /* 0x1FU = 31 bits max shift */
    pHandle->timer.Instance->CCER |=
    (uint32_t)(TIM_CCx_ENABLE << (pHandle->channelID & 0x1FU));

    if (IS_TIM_BREAK_INSTANCE (pHandle->timer.Instance) != FALSE) {
        /* Enable the Main Output */
        pHandle->timer.Instance->BDTR |= TIM_BDTR_MOE;
    }
    pHandle->timer.Instance->CR1 |= TIM_CR1_CEN;
    return eSTATUS_SUCCESS;
}

eSTATUS_t PWMStop (PWMHandle* pHandle) {
    if (PWM_CHECK_OK (pHandle) == FALSE) {
        LOG_ERROR ("Received invalid PWM pointer");
        return eSTATUS_FAILURE;
    }

    if (HAL_TIM_PWM_Stop (&pHandle->timer, pHandle->channelID) != HAL_OK) {
        LOG_ERROR ("Failed to stop PWM");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t PWMWrite (PWMHandle* pHandle, uint32_t usUpTime) {

    if (PWM_CHECK_OK (pHandle) == FALSE) {
        LOG_ERROR ("Received invalid PWM pointer");
        return eSTATUS_FAILURE;
    }

    /* ARR is synonymous with microseconds because the clock is prescaled to 1MHz */
    /* Set ARR and PSC here because sometimes they don't get set properly */
    pHandle->timer.Instance->ARR = pHandle->timer.Init.Period;
    pHandle->timer.Instance->PSC = pHandle->timer.Init.Prescaler;

    PWM_SET_COMPARE (pHandle, usUpTime);
    return eSTATUS_SUCCESS;
}

eSTATUS_t PWM_DMAInit (PWM_DMAConfig timConfig, DMAConfig dmaConfig, PWM_DMAHandle* pOutHandle) {

    if (pOutHandle == NULL) {
        LOG_ERROR ("Received nullptr for PWM_DMAHandle");
        return eSTATUS_FAILURE;
    }

    if (PWMDMA_CHECK_CONF_OK (&timConfig) == FALSE) {
        LOG_ERROR ("Received invalid PWM DMA configuration pointer or timer pointer");
        return eSTATUS_FAILURE;
    }

    memset (pOutHandle, 0, sizeof (PWM_DMAHandle));
    PWMConfig pwmConfig = { .base = timConfig.base };
    eSTATUS_t status    = PWMInit (pwmConfig, &pOutHandle->pwm);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize PWM for DMA");
        return status;
    }

    status = DMA_Init (dmaConfig, &pOutHandle->pDMA);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize DMA");
        return status;
    }

    if (timConfig.dmaRegIDXCount > MAX_DMA_TIM_REGISTERS ||
        timConfig.dmaRegIDXCount == 0) {
        LOG_ERROR (
        "Invalid DMA register index count: %u",
        (uint16_t)timConfig.dmaRegIDXCount
        );
        return eSTATUS_FAILURE;
    }

    for (uint16_t i = 0; i < timConfig.dmaRegIDXCount; ++i) {
        if (timConfig.dmaRegIDXs[i] >= MAX_DMA_TIM_REGISTERS) {
            LOG_ERROR ("Invalid DMA register index: %u", timConfig.dmaRegIDXs[i]);
            return eSTATUS_FAILURE;
        }
        /*
         * Link the DMA handle to the specific PWM timer register it is
         * going to be writing to.
         */
        pOutHandle->pwm.timer.hdma[timConfig.dmaRegIDXs[i]] = pOutHandle->pDMA;
    }
    // Link the DMA handle to the parent timer
    pOutHandle->pDMA->Parent = &pOutHandle->pwm.timer;
    return eSTATUS_SUCCESS;
}

eSTATUS_t PWM_DMAStart (PWM_DMAHandle* pHandle, uint32_t const* pData, uint16_t Length) {

    if (HAL_TIM_PWM_Start_DMA (&pHandle->pwm.timer, pHandle->pwm.channelID, (uint32_t*)pData, Length) !=
        HAL_OK) {
        LOG_ERROR ("Failed to start PWM DMA");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t
PWM_DMARegisterCallback (PWM_DMAHandle* pHandle, uint32_t channelID, PWM_DMACallback callback, ePWM_DMA_CB_TYPE cbType) {

    if (pHandle == NULL || callback == NULL) {
        LOG_ERROR ("Invalid parameters for PWM DMA callback registration");
        return eSTATUS_FAILURE;
    }

    int32_t id     = PWMTimChannel2Idx (channelID);
    int32_t timIdx = PWMTim2Idx (pHandle->pwm.timer.Instance);

    if (timIdx < 0 || id < 0) {
        LOG_ERROR ("Invalid timer index or channel ID for PWM DMA callback registration");
        return eSTATUS_FAILURE;
    }

    gPWM_DMACallbacks[timIdx][cbType][id] = callback;
    return eSTATUS_SUCCESS;
}