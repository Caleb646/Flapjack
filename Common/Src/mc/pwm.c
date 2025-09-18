#include "mc/pwm.h"
#include "common.h"
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

static eSTATUS_t PWMEnableTimClock (PWMHandle* pHandle) {

    if (pHandle == NULL || pHandle->timer.Instance == NULL) {
        LOG_ERROR ("Invalid PWM handle or timer instance");
        return eSTATUS_FAILURE;
    }

    TIM_TypeDef* pTimerRegisters = pHandle->timer.Instance;

    if (pTimerRegisters == TIM8) {
        LOG_INFO ("Enabling clock for TIM8");
        __HAL_RCC_TIM8_CLK_ENABLE ();
    } else if (pTimerRegisters == TIM13) {
        LOG_INFO ("Enabling clock for TIM13");
        __HAL_RCC_TIM13_CLK_ENABLE ();
    } else {
        LOG_ERROR ("Invalid timer registers pointer");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

// eSTATUS_t PWMInitTimBaseConfig (PWMHandle* pHandle) {

//     if (pHandle == NULL || pHandle->timer.Instance == NULL) {
//         return eSTATUS_FAILURE;
//     }

//     TIM_TypeDef* pTimerRegisters = pHandle->timer.Instance;

//     if (PWMEnableTimClock (pHandle) != eSTATUS_SUCCESS) {
//         LOG_ERROR ("Failed to enable timer clock");
//         return eSTATUS_FAILURE;
//     }

//     uint32_t tmpcr1 = pTimerRegisters->CR1;
//     /* Set the Autoreload value 0 to 65535 */
//     pTimerRegisters->ARR = pHandle->timer.Init.Period;
//     /* Set the Prescaler value 0 to 65535 */
//     pTimerRegisters->PSC = pHandle->timer.Init.Prescaler;

//     /* Set the counter mode */
//     // if (IS_TIM_COUNTER_MODE_SELECT_INSTANCE (pTimerRegisters)) {
//     tmpcr1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);
//     tmpcr1 |= TIM_COUNTERMODE_UP;
//     // }

//     /* Set the clock division */
//     if (IS_TIM_CLOCK_DIVISION_INSTANCE (pTimerRegisters)) {
//         tmpcr1 &= ~TIM_CR1_CKD;
//         tmpcr1 |= (uint32_t)TIM_CLOCKDIVISION_DIV1;
//     }

//     /* Set the auto-reload preload to ENABLED */
//     if (pHandle->timer.Init.AutoReloadPreload == TIM_AUTORELOAD_PRELOAD_ENABLE) {
//         pTimerRegisters->CR1 |= TIM_CR1_ARPE;
//     } else {
//         pTimerRegisters->CR1 &= ~TIM_CR1_ARPE;
//     }

//     if (IS_TIM_REPETITION_COUNTER_INSTANCE (pTimerRegisters)) {
//         /* Set the Repetition Counter value */
//         pTimerRegisters->RCR = 0U;
//     }
//     /* Enable Update Event (UEV) by clearing Update Disable (UDIS) bit */
//     /*
//      * The UEV (update event) event can be disabled by software by setting
//      * the UDIS bit in the TIMx_CR1 register. This is to avoid updating the
//      * shadow registers while writing new values in the preload registers.
//      * Then no update event occurs until the UDIS bit has been written to
//      * 0. However, the counter restarts from 0, as well as the counter of
//      * the prescaler (but the prescale rate does not change). In addition,
//      * if the URS bit (update request selection) in TIMx_CR1 register is
//      * set, setting the UG bit generates an update event UEV but without
//      * setting the UIF flag (thus no interrupt is sent). This is to avoid
//      * generating both update and capture interrupts when clearing the
//      * counter on the capture event.
//      */
//     pTimerRegisters->CR1 |= TIM_CR1_URS;
//     pTimerRegisters->EGR |= TIM_EGR_UG;
//     pTimerRegisters->CR1 &= ~TIM_CR1_UDIS;
//     /* UIF flag shouldn't be set but to avoid spurious interrupts set it to 0 */
//     // Ensure the EGR is processed
//     // clang-format off
//     while ((pTimerRegisters->EGR & TIM_EGR_UG) == SET);
//     // clang-format on
//     pTimerRegisters->SR  = ~TIM_SR_UIF; /* Clear update flag. */
//     pTimerRegisters->CR1 = tmpcr1;

//     pHandle->timer.DMABurstState = HAL_DMA_BURST_STATE_READY;
//     TIM_CHANNEL_STATE_SET_ALL (&pHandle->timer, HAL_TIM_CHANNEL_STATE_READY);
//     TIM_CHANNEL_N_STATE_SET_ALL (&pHandle->timer, HAL_TIM_CHANNEL_STATE_READY);
//     pHandle->timer.State = HAL_TIM_STATE_READY;

//     return eSTATUS_SUCCESS;
// }

// eSTATUS_t PWMInitChannel (PWMHandle* pHandle) {

//     if (PWM_CHECK_OK (pHandle) == FALSE) {
//         return eSTATUS_FAILURE;
//     }

//     uint32_t timerChannelID      = pHandle->channelID;
//     TIM_TypeDef* pTimerRegisters = pHandle->timer.Instance;
//     if (timerChannelID == TIM_CHANNEL_1 && IS_TIM_CC1_INSTANCE (pTimerRegisters)) {
//         uint32_t tmpccmrx;
//         uint32_t tmpccer;
//         uint32_t tmpcr2;
//         /* Get the TIMx CCER register value */
//         tmpccer = pTimerRegisters->CCER;
//         /* Disable the Channel 1: Reset the CC1E Bit */
//         pTimerRegisters->CCER &= ~TIM_CCER_CC1E;
//         /* Get the TIMx CR2 register value */
//         tmpcr2 = pTimerRegisters->CR2;
//         /* Get the TIMx CCMR1 register value */
//         tmpccmrx = pTimerRegisters->CCMR1;
//         /* Reset the Output Compare Mode Bits */
//         tmpccmrx &= ~TIM_CCMR1_OC1M;
//         tmpccmrx &= ~TIM_CCMR1_CC1S;
//         /* Select the Output Compare Mode */
//         tmpccmrx |= TIM_OCMODE_PWM1;
//         /* Reset the Output Polarity level */
//         tmpccer &= ~TIM_CCER_CC1P;
//         /* Set the Output Compare Polarity */
//         tmpccer |= TIM_OCPOLARITY_HIGH;

//         if (IS_TIM_CCXN_INSTANCE (pTimerRegisters, TIM_CHANNEL_1)) {
//             /* Reset the Output N Polarity level */
//             tmpccer &= ~TIM_CCER_CC1NP;
//             /* Set the Output N Polarity */
//             tmpccer |= TIM_OCNPOLARITY_HIGH;
//             /* Reset the Output N State */
//             tmpccer &= ~TIM_CCER_CC1NE;
//         }

//         if (IS_TIM_BREAK_INSTANCE (pTimerRegisters)) {
//             /* Reset the Output Compare and Output Compare N IDLE State
//             */ tmpcr2 &= ~TIM_CR2_OIS1; tmpcr2 &= ~TIM_CR2_OIS1N;
//             /* Set the Output Idle state */
//             tmpcr2 |= TIM_OCIDLESTATE_RESET;
//             /* Set the Output N Idle state */
//             tmpcr2 |= TIM_OCNIDLESTATE_RESET;
//         }

//         /* Write to TIMx CR2 */
//         pTimerRegisters->CR2 = tmpcr2;
//         /* Write to TIMx CCMR1 */
//         pTimerRegisters->CCMR1 = tmpccmrx;
//         /* Set the Capture Compare Register value */
//         pTimerRegisters->CCR1 = 0;
//         /* Write to TIMx CCER */
//         pTimerRegisters->CCER = tmpccer;
//         /* Set the Preload enable bit for channel1 */
//         pTimerRegisters->CCMR1 |= TIM_CCMR1_OC1PE;
//         /* Configure the Output Fast mode */
//         pTimerRegisters->CCMR1 &= ~TIM_CCMR1_OC1FE;
//         pTimerRegisters->CCMR1 |= TIM_OCFAST_DISABLE;

//     } else if (timerChannelID == TIM_CHANNEL_2 && IS_TIM_CC2_INSTANCE (pTimerRegisters)) {
//         uint32_t tmpccmrx;
//         uint32_t tmpccer;
//         uint32_t tmpcr2;

//         /* Get the TIMx CCER register value */
//         tmpccer = pTimerRegisters->CCER;
//         /* Disable the Channel 2: Reset the CC2E Bit */
//         pTimerRegisters->CCER &= ~TIM_CCER_CC2E;
//         /* Get the TIMx CR2 register value */
//         tmpcr2 = pTimerRegisters->CR2;
//         /* Get the TIMx CCMR1 register value */
//         tmpccmrx = pTimerRegisters->CCMR1;
//         /* Reset the Output Compare mode and Capture/Compare selection Bits */
//         tmpccmrx &= ~TIM_CCMR1_OC2M;
//         tmpccmrx &= ~TIM_CCMR1_CC2S;
//         /* Select the Output Compare Mode */
//         tmpccmrx |= (TIM_OCMODE_PWM1 << 8U);
//         /* Reset the Output Polarity level */
//         tmpccer &= ~TIM_CCER_CC2P;
//         /* Set the Output Compare Polarity */
//         tmpccer |= (TIM_OCPOLARITY_HIGH << 4U);

//         if (IS_TIM_CCXN_INSTANCE (pTimerRegisters, TIM_CHANNEL_2)) {
//             /* Reset the Output N Polarity level */
//             tmpccer &= ~TIM_CCER_CC2NP;
//             /* Set the Output N Polarity */
//             tmpccer |= (TIM_OCNPOLARITY_HIGH << 4U);
//             /* Reset the Output N State */
//             tmpccer &= ~TIM_CCER_CC2NE;
//         }

//         if (IS_TIM_BREAK_INSTANCE (pTimerRegisters)) {
//             /* Reset the Output Compare and Output Compare N IDLE State
//             */ tmpcr2 &= ~TIM_CR2_OIS2; tmpcr2 &= ~TIM_CR2_OIS2N;
//             /* Set the Output Idle state */
//             tmpcr2 |= (TIM_OCIDLESTATE_RESET << 2U);
//             /* Set the Output N Idle state */
//             tmpcr2 |= (TIM_OCNIDLESTATE_RESET << 2U);
//         }

//         /* Write to TIMx CR2 */
//         pTimerRegisters->CR2 = tmpcr2;
//         /* Write to TIMx CCMR1 */
//         pTimerRegisters->CCMR1 = tmpccmrx;
//         /* Set the Capture Compare Register value */
//         pTimerRegisters->CCR2 = 0U;
//         /* Write to TIMx CCER */
//         pTimerRegisters->CCER = tmpccer;
//         /* Set the Preload enable bit for channel2 */
//         pTimerRegisters->CCMR1 |= TIM_CCMR1_OC2PE;
//         /* Configure the Output Fast mode */
//         pTimerRegisters->CCMR1 &= ~TIM_CCMR1_OC2FE;
//         pTimerRegisters->CCMR1 |= TIM_OCFAST_DISABLE << 8U;
//     } else {
//         LOG_ERROR ("Unsupported timer or timer channel id [0x%X] for PWM initialization", (uint16_t)timerChannelID);
//         return eSTATUS_FAILURE;
//     }

//     return eSTATUS_SUCCESS;
// }

static eSTATUS_t PWMInitGPIO (PWMHandle* pHandle) {

    if (PWM_CHECK_OK (pHandle) == FALSE) {
        LOG_ERROR ("Received invalid PWM handle or timer registers pointer");
        return eSTATUS_FAILURE;
    }

    if (GPIOInitTimer (pHandle->timer.Instance, pHandle->channelID) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize GPIO for timer");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t PWMInit (PWMConfig config, PWMHandle* pOutHandle) {

    if (pOutHandle == NULL) {
        LOG_ERROR ("PWM output handle pointer is NULL");
        return eSTATUS_FAILURE;
    }

    if (PWM_CHECK_CONF_OK (&config) == FALSE) {
        LOG_ERROR ("Received invalid PWM configuration pointer or timer pointer");
        return eSTATUS_FAILURE;
    }

    memset (pOutHandle, 0, sizeof (PWMHandle));
    uint32_t channelID  = config.base.channelID;
    uint32_t autoReload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    uint32_t prescaler  = (uint16_t)(SystemCoreClock / 1000000U) - 1U;
    uint32_t period = (uint16_t)(1000000U / MAX_U32 (config.base.hzPeriod, 1U)) - 1U;

    if (config.base.doAutoReload == FALSE) {
        LOG_INFO ("Enabling auto-reload for PWM timer");
        autoReload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    }

    pOutHandle->timer.Instance               = config.base.pTimer;
    pOutHandle->channelID                    = channelID;
    pOutHandle->timer.Init.Prescaler         = prescaler;
    pOutHandle->timer.Init.Period            = period;
    pOutHandle->timer.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    pOutHandle->timer.Init.CounterMode       = TIM_COUNTERMODE_UP;
    pOutHandle->timer.Init.RepetitionCounter = 0;
    pOutHandle->timer.Init.AutoReloadPreload = autoReload;

    eSTATUS_t status = eSTATUS_SUCCESS;
    status           = PWMEnableTimClock (pOutHandle);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to enable timer clock");
        return status;
    }

    if (HAL_TIM_PWM_Init (&pOutHandle->timer) != HAL_OK) {
        LOG_ERROR ("Failed to initialize timer PWM");
        return eSTATUS_FAILURE;
    }

    TIM_OC_InitTypeDef sConfig = { 0 };
    sConfig.OCMode             = TIM_OCMODE_PWM1;
    sConfig.OCPolarity         = TIM_OCPOLARITY_HIGH;
    sConfig.Pulse              = 0;
    sConfig.OCNPolarity        = TIM_OCNPOLARITY_HIGH;
    sConfig.OCFastMode         = TIM_OCFAST_DISABLE;
    sConfig.OCIdleState        = TIM_OCIDLESTATE_RESET;
    sConfig.OCNIdleState       = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel (&pOutHandle->timer, &sConfig, channelID) != HAL_OK) {
        LOG_ERROR ("Failed to configure PWM channel");
        return eSTATUS_FAILURE;
    }

    status = PWMInitGPIO (pOutHandle);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize GPIO for timer");
        return status;
    }

    return eSTATUS_SUCCESS;
}

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