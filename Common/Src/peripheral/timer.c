#include "peripheral/timer.h"
#include "conf/conf.h"
#include "core/core.h"
#include "core/log/logger.h"
#include "hal.h"
#include "mem/mem.h"
#include "peripheral/dma.h"
#include "peripheral/gpio.h"
#include <stdint.h>
#include <string.h>



#define TIMER_DO_CALLBACK(pTIMER, CB_FN)     \
    do {                                     \
        if ((CB_FN) != NULL) {               \
            (CB_FN) (TIMER_GET_ID (pTIMER)); \
        }                                    \
    } while (0)
#define TIMER_VALID(pTIMER)                                                                        \
    (                                                                                              \
    (pTIMER) != NULL && TIMER_GET_DEVICE (pTIMER) != NULL &&                                       \
    (pTIMER)->isTimerInitialized == true && TIMER_GET_DEVICE (pTIMER)->isDeviceInitialized == true \
    )

static SHARED_MEM_SECTION TimerDevice_t g_Devices[TIMER_MAX_DEVICES] = { 0 };
static SHARED_MEM_SECTION Timer_t g_Timers[TIMER_MAX_TIMERS]         = { 0 };

static vTimer_t* Timer_Get_ByInstance (TIM_TypeDef* instance, uint32_t channelIdx);
static void TimerHandleCallback (TIM_HandleTypeDef* htim, eTIMER_CALLBACK_ID_t callbackType);
static eSTATUS_t Timer_DMAInit (vTimer_t* pTimer, TimerInitConf_t conf);

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

    if (((int32_t)htim->Channel - 1) < 0) {
        return;
    }

    vTimer_t* pTimer = Timer_Get_ByInstance (htim->Instance, htim->Channel - 1U);

    // clang-format off
    switch (callbackType) {
    case eTIMER_CALLBACK_TRANSFER_COMPLETE: TIMER_DO_CALLBACK (pTimer, pTimer->pulseFinishedCallback); break;
    case eTIMER_CALLBACK_TRANSFER_ERROR: TIMER_DO_CALLBACK (pTimer, pTimer->errorCallback); break;
    default: break;
    }
    // clang-format on
}

static vTimerDevice_t* Timer_Get_DeviceById (eTIMER_ID_t timerId) {

    uint32_t timerIdx = TIMER_ID_TO_DEVICE_IDX (timerId);
    if (timerIdx >= TIMER_MAX_DEVICES) {
        LOG_ERROR ("Invalid timer index: %u", (uint16_t)timerIdx);
        return NULL;
    }
    return &g_Devices[timerIdx];
}

static vTimer_t* Timer_Get_ByInstance (TIM_TypeDef* instance, uint32_t channelIdx) {

    for (uint32_t i = 0; i < TIMER_MAX_TIMERS; ++i) {
        if (TIMER_GET_INSTANCE (&g_Timers[i]) == instance && TIMER_GET_CHANNEL (&g_Timers[i]) == channelIdx) {
            return &g_Timers[i];
        }
    }
    return NULL;
}

static TIM_TypeDef* Timer_Get_InstanceById (eTIMER_ID_t timerId) {

    switch (TIMER_ID_CLR_CHAN_BITS (timerId)) {
    case eTIMER_5_DEVICE_ID: return TIM5;
    case eTIMER_8_DEVICE_ID: return TIM8;
    case eTIMER_12_DEVICE_ID: return TIM12;
    case eTIMER_13_DEVICE_ID: return TIM13;
    default: LOG_ERROR ("Invalid timer ID: %u", (uint16_t)timerId); return NULL;
    }
}

/*
 * DMA is NOT supported on all timers
 */
static uint32_t Timer_Get_DMARequestId (eTIMER_ID_t timerId) {

    uint32_t channelIdx = TIMER_ID_TO_CHAN_IDX (timerId);
    switch (TIMER_ID_CLR_CHAN_BITS (timerId)) {
    case eTIMER_5_DEVICE_ID: return DMA_REQUEST_TIM5_CH1 + channelIdx;
    case eTIMER_8_DEVICE_ID: return DMA_REQUEST_TIM8_CH1 + channelIdx;
    default: LOG_ERROR ("Invalid timer ID for DMA request: %u", (uint16_t)timerId); return 0;
    }
}

static eSTATUS_t Timer_BaseInit (vTimer_t* pTimer, TimerInitConf_t conf) {

    RETURN_IF (pTimer->isTimerInitialized == true, eSTATUS_FAILURE, "Timer already initialized");

    TimerBoardConf_t boardConf = conf.timerBoardConf;
    eTIMER_ID_t timerId        = boardConf.timerId;
    eDEVICE_ID_t deviceId      = conf.deviceId;
    eTIMER_MODE_t mode         = conf.mode;
    bool usingDMA              = conf.usingDMA;

    memset (pTimer, 0, sizeof (vTimer_t));
    pTimer->deviceId = deviceId;
    pTimer->timerId  = timerId;
    pTimer->mode     = mode;
    pTimer->usingDMA = usingDMA;
    pTimer->pDevice  = Timer_Get_DeviceById (timerId);
    GOTO_IF_NULL (pTimer->pDevice, error, "Failed to get timer device");
    return eSTATUS_SUCCESS;

error:
    memset (pTimer, 0, sizeof (vTimer_t));
    return eSTATUS_FAILURE;
}

static eSTATUS_t Timer_ClockInit (vTimer_t* pTimer, TimerInitConf_t conf) {

    FJ_UNUSED (conf);
    eTIMER_ID_t timerId = TIMER_GET_ID (pTimer);
    switch (TIMER_ID_CLR_CHAN_BITS (timerId)) {
    case eTIMER_5_DEVICE_ID: __HAL_RCC_TIM5_CLK_ENABLE (); break;
    case eTIMER_8_DEVICE_ID: __HAL_RCC_TIM8_CLK_ENABLE (); break;
    case eTIMER_12_DEVICE_ID: __HAL_RCC_TIM12_CLK_ENABLE (); break;
    case eTIMER_13_DEVICE_ID: __HAL_RCC_TIM13_CLK_ENABLE (); break;
    default: LOG_ERROR ("Invalid timer ID: %u", (uint16_t)timerId); return eSTATUS_INVALID_ARG;
    }
    return eSTATUS_SUCCESS;
}

static eSTATUS_t Timer_DeviceInit (vTimer_t* pTimer, TimerInitConf_t conf) {

    if (TIMER_GET_DEVICE (pTimer)->isDeviceInitialized == true) {
        return eSTATUS_SUCCESS;
    }

    eSTATUS_t status = Timer_ClockInit (pTimer, conf);
    GOTO_IF (STATUS_FAIL (status), error, "Failed to initialize timer clock");

    eTIMER_ID_t timerId = TIMER_GET_ID (pTimer);
    uint32_t autoReload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    uint32_t prescaler  = (SystemCoreClock / 1000000U) - 1U;
    uint32_t period     = (1000000U / MAX_U32 (conf.hzPeriod, 1U)) - 1U;

    if (conf.doAutoPreload == false) {
        LOG_INFO ("Disabling auto-reload for PWM timer");
        autoReload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    }

    TIMER_GET_HANDLE (pTimer)->Instance               = Timer_Get_InstanceById (timerId);
    TIMER_GET_HANDLE (pTimer)->Init.Prescaler         = prescaler;
    TIMER_GET_HANDLE (pTimer)->Init.Period            = period;
    TIMER_GET_HANDLE (pTimer)->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    TIMER_GET_HANDLE (pTimer)->Init.CounterMode       = TIM_COUNTERMODE_UP;
    TIMER_GET_HANDLE (pTimer)->Init.RepetitionCounter = 0;
    TIMER_GET_HANDLE (pTimer)->Init.AutoReloadPreload = autoReload;

    if (conf.mode == eTIMER_MODE_PWM) {
        status = HAL_TIM_PWM_Init (TIMER_GET_HANDLE (pTimer));
        GOTO_IF (STATUS_FAIL (status), error, "Failed to initialize timer device in PWM mode");
    } else {
        LOG_ERROR ("Unsupported timer mode: %u", (uint16_t)conf.mode);
        goto error;
    }

    TIMER_GET_DEVICE (pTimer)->isDeviceInitialized = true;
    return eSTATUS_SUCCESS;

error:
    memset (TIMER_GET_DEVICE (pTimer), 0, sizeof (TimerDevice_t));
    return eSTATUS_FAILURE;
}

static eSTATUS_t Timer_DMAInit (vTimer_t* pTimer, TimerInitConf_t conf) {

    FJ_UNUSED (conf);
    eSTATUS_t status    = eSTATUS_SUCCESS;
    eTIMER_ID_t timerId = TIMER_GET_ID (pTimer);
    bool usingDMA       = pTimer->usingDMA;
    if (usingDMA == false) {
        return eSTATUS_FAILURE;
    }
    eDMA_STREAM_ID_t dmaStreamId = eDMA_STREAM_MAX;
    uint32_t dmaRequestId        = Timer_Get_DMARequestId (timerId);
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
    TIM_HandleTypeDef* pHandle = TIMER_GET_HANDLE (pTimer);
    uint32_t channelIdx        = TIMER_ID_TO_CHAN_IDX (timerId);
    switch (channelIdx) {
    case eTIMER_CHANNEL_1_ID: pHandle->hdma[TIM_DMA_ID_CC1] = &pDMA->handle; break;
    case eTIMER_CHANNEL_2_ID: pHandle->hdma[TIM_DMA_ID_CC2] = &pDMA->handle; break;
    case eTIMER_CHANNEL_3_ID: pHandle->hdma[TIM_DMA_ID_CC3] = &pDMA->handle; break;
    case eTIMER_CHANNEL_4_ID: pHandle->hdma[TIM_DMA_ID_CC4] = &pDMA->handle; break;
    default: LOG_ERROR ("Invalid timer channel index: %u", channelIdx); return eSTATUS_FAILURE;
    }
    // Link the DMA handle to the parent timer
    pDMA->handle.Parent = (void*)pHandle;
    return eSTATUS_SUCCESS;
}

static eSTATUS_t Timer_GPIOInit (vTimer_t* pTimer, TimerInitConf_t conf) {

    FJ_UNUSED (pTimer);
    eSTATUS_t status                      = eSTATUS_SUCCESS;
    TimerBoardConf_t boardConf            = conf.timerBoardConf;
    eTIMER_ID_t timerId                   = boardConf.timerId;
    GPIOBoardConf_t const* pGPIOBoardConf = boardConf.pGPIOBoardConf;
    GPIO_INIT (&status, timerId, *pGPIOBoardConf);
    RETURN_IF (STATUS_FAIL (status), status, "Failed to initialize timer GPIO");

    return eSTATUS_SUCCESS;
}

vTimer_t* Timer_Get_ById (eTIMER_ID_t timerId) {

    uint32_t timerIdx = TIMER_ID_TO_IDX (timerId);
    if (timerIdx >= TIMER_MAX_TIMERS) {
        LOG_ERROR ("Invalid timer index: %u", (uint16_t)timerIdx);
        return NULL;
    }
    return &g_Timers[timerIdx];
}

eSTATUS_t Timer_Init (TimerInitConf_t conf, vTimer_t* pOutTimer) {

    eSTATUS_t status           = eSTATUS_SUCCESS;
    TimerBoardConf_t boardConf = conf.timerBoardConf;
    eTIMER_ID_t timerId        = boardConf.timerId;
    bool usingDMA              = conf.usingDMA;

    vTimer_t* pTimer = Timer_Get_ById (timerId);
    if (pOutTimer != NULL) {
        pTimer = pOutTimer;
    }

    status = Timer_BaseInit (pTimer, conf);
    GOTO_IF (STATUS_FAIL (status), error, "Failed to do base timer initialization");

    status = Timer_DeviceInit (pTimer, conf);
    GOTO_IF (STATUS_FAIL (status), error, "Failed to initialize timer device");

    TIM_OC_InitTypeDef sConfig = { 0 };
    sConfig.OCMode             = TIM_OCMODE_PWM1;
    sConfig.OCPolarity         = TIM_OCPOLARITY_HIGH;
    sConfig.Pulse              = 0;
    sConfig.OCNPolarity        = TIM_OCNPOLARITY_HIGH;
    sConfig.OCFastMode         = TIM_OCFAST_DISABLE;
    sConfig.OCIdleState        = TIM_OCIDLESTATE_RESET;
    sConfig.OCNIdleState       = TIM_OCNIDLESTATE_RESET;

    if (HAL_TIM_PWM_ConfigChannel (TIMER_GET_HANDLE (pTimer), &sConfig, TIMER_ID_TO_HAL_CHAN (timerId)) != HAL_OK) {
        LOG_ERROR ("Failed to configure PWM channel");
        goto error;
    }

    if (usingDMA == true) {
        if (Timer_DMAInit (pTimer, conf) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize Timer DMA");
            goto error;
        }
    }

    if (Timer_GPIOInit (pTimer, conf) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize timer GPIO");
        goto error;
    }

    pTimer->isTimerInitialized = true;
    return eSTATUS_SUCCESS;

error:
    memset (pTimer, 0, sizeof (vTimer_t));
    return eSTATUS_FAILURE;
}

eSTATUS_t Timer_Start (Timer_t* pTimer, uint32_t const* pData, uint16_t Length) {

    if (TIMER_VALID (pTimer) == false) {
        LOG_ERROR ("Invalid timer or timer not initialized");
        return eSTATUS_INVALID_ARG;
    }

    TIM_HandleTypeDef* pHandle = TIMER_GET_HANDLE (pTimer);
    eTIMER_MODE_t mode         = pTimer->mode;
    bool usingDMA              = pTimer->usingDMA;
    uint32_t channel           = TIMER_ID_TO_HAL_CHAN (TIMER_GET_ID (pTimer));

    if (usingDMA == true && mode == eTIMER_MODE_PWM) {

        if (pData == NULL || Length == 0) {
            LOG_ERROR ("Invalid data or length for PWM DMA start");
            return eSTATUS_INVALID_ARG;
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
        return eSTATUS_INVALID_ARG;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t Timer_Stop (Timer_t* pTimer) {

    if (TIMER_VALID (pTimer) == false) {
        LOG_ERROR ("Invalid timer or timer not initialized");
        return eSTATUS_INVALID_ARG;
    }

    TIM_HandleTypeDef* pHandle = TIMER_GET_HANDLE (pTimer);
    eTIMER_MODE_t mode         = pTimer->mode;
    bool usingDMA              = pTimer->usingDMA;
    uint32_t channel           = TIMER_ID_TO_HAL_CHAN (TIMER_GET_ID (pTimer));

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

eSTATUS_t Timer_Write (Timer_t* pTimer, uint32_t usUpTime) {

    if (TIMER_VALID (pTimer) == false) {
        LOG_ERROR ("Invalid timer or timer not initialized");
        return eSTATUS_INVALID_ARG;
    }
    /* ARR is synonymous with microseconds because the clock is prescaled to 1MHz */
    /* Set ARR and PSC here because sometimes they don't get set properly */

    TIMER_SET_PRESCALER (pTimer, TIMER_GET_HANDLE (pTimer)->Init.Prescaler);
    TIMER_SET_PERIOD (pTimer, TIMER_GET_HANDLE (pTimer)->Init.Period);
    TIMER_SET_COMPARE (pTimer, usUpTime);

    return eSTATUS_SUCCESS;
}

eSTATUS_t Timer_RegisterCallback (Timer_t* pTimer, TimerCallback_t callback, eTIMER_CALLBACK_ID_t cbType) {

    if (TIMER_VALID (pTimer) == false) {
        LOG_ERROR ("Invalid timer or timer not initialized");
        return eSTATUS_INVALID_ARG;
    }

    if (callback == NULL) {
        LOG_ERROR ("Invalid parameters for Timer callback registration");
        return eSTATUS_INVALID_ARG;
    }

    switch (cbType) {
    case eTIMER_CALLBACK_TRANSFER_COMPLETE: pTimer->pulseFinishedCallback = callback; break;
    case eTIMER_CALLBACK_TRANSFER_ERROR: pTimer->errorCallback = callback; break;
    default: LOG_ERROR ("Unsupported callback type"); return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eTIMER_CHANNEL_STATE_t Timer_GetChannelState (Timer_t* pTimer) {

    if (TIMER_VALID (pTimer) == false) {
        LOG_ERROR ("Failed to get timer by ID");
        return eTIMER_CHANNEL_STATE_ERROR;
    }

    HAL_TIM_ChannelStateTypeDef channelState =
    TIM_CHANNEL_STATE_GET (TIMER_GET_HANDLE (pTimer), TIMER_ID_TO_HAL_CHAN (TIMER_GET_ID (pTimer)));

    switch (channelState) {
    case HAL_TIM_CHANNEL_STATE_RESET: return eTIMER_CHANNEL_STATE_RESET;
    case HAL_TIM_CHANNEL_STATE_READY: return eTIMER_CHANNEL_STATE_READY;
    case HAL_TIM_CHANNEL_STATE_BUSY: return eTIMER_CHANNEL_STATE_BUSY;
    }
    return eTIMER_CHANNEL_STATE_ERROR;
}