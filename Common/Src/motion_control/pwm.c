#include "motion_control/pwm.h"
#include "common.h"
#include "dma.h"
#include "hal.h"
#include "log.h"
#include <stdint.h>

#define MAX_DMA_TIM_REGISTERS 7U


STATUS_TYPE PWMInitTimBaseConfig (PWMHandle* pHandle) {

    if (pHandle == NULL || pHandle->timer.Instance == NULL) {
        return eSTATUS_FAILURE;
    }

    TIM_TypeDef* pTimerRegisters = pHandle->timer.Instance;

    if (pTimerRegisters == TIM8) {
        __HAL_RCC_TIM8_CLK_ENABLE ();
    } else if (pTimerRegisters == TIM13) {
        __HAL_RCC_TIM13_CLK_ENABLE ();
    } else {
        LOG_ERROR ("Invalid timer registers pointer");
        return eSTATUS_FAILURE;
    }

    uint32_t tmpcr1 = pTimerRegisters->CR1;
    /* Set the Autoreload value 0 to 65535 */
    pTimerRegisters->ARR = pHandle->timer.Init.Period;
    /* Set the Prescaler value 0 to 65535 */
    pTimerRegisters->PSC = pHandle->timer.Init.Prescaler;

    /* Set the counter mode */
    // if (IS_TIM_COUNTER_MODE_SELECT_INSTANCE (pTimerRegisters)) {
    tmpcr1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);
    tmpcr1 |= TIM_COUNTERMODE_UP;
    // }

    /* Set the clock division */
    if (IS_TIM_CLOCK_DIVISION_INSTANCE (pTimerRegisters)) {
        tmpcr1 &= ~TIM_CR1_CKD;
        tmpcr1 |= (uint32_t)TIM_CLOCKDIVISION_DIV1;
    }

    /* Set the auto-reload preload to ENABLED */
    if (pHandle->timer.Init.AutoReloadPreload == TIM_AUTORELOAD_PRELOAD_ENABLE) {
        pTimerRegisters->CR1 |= TIM_CR1_ARPE;
    } else {
        pTimerRegisters->CR1 &= ~TIM_CR1_ARPE;
    }

    if (IS_TIM_REPETITION_COUNTER_INSTANCE (pTimerRegisters)) {
        /* Set the Repetition Counter value */
        pTimerRegisters->RCR = 0U;
    }
    /* Enable Update Event (UEV) by clearing Update Disable (UDIS) bit */
    /*
     * The UEV (update event) event can be disabled by software by setting
     * the UDIS bit in the TIMx_CR1 register. This is to avoid updating the
     * shadow registers while writing new values in the preload registers.
     * Then no update event occurs until the UDIS bit has been written to
     * 0. However, the counter restarts from 0, as well as the counter of
     * the prescaler (but the prescale rate does not change). In addition,
     * if the URS bit (update request selection) in TIMx_CR1 register is
     * set, setting the UG bit generates an update event UEV but without
     * setting the UIF flag (thus no interrupt is sent). This is to avoid
     * generating both update and capture interrupts when clearing the
     * counter on the capture event.
     */
    pTimerRegisters->CR1 |= TIM_CR1_URS;
    pTimerRegisters->EGR |= TIM_EGR_UG;
    pTimerRegisters->CR1 &= ~TIM_CR1_UDIS;
    /* UIF flag shouldn't be set but to avoid spurious interrupts set it to 0 */
    // Ensure the EGR is processed
    // clang-format off
    while ((pTimerRegisters->EGR & TIM_EGR_UG) == SET);
    // clang-format on
    pTimerRegisters->SR  = ~TIM_SR_UIF; /* Clear update flag. */
    pTimerRegisters->CR1 = tmpcr1;

    pHandle->timer.DMABurstState = HAL_DMA_BURST_STATE_READY;
    TIM_CHANNEL_STATE_SET_ALL (&pHandle->timer, HAL_TIM_CHANNEL_STATE_READY);
    TIM_CHANNEL_N_STATE_SET_ALL (&pHandle->timer, HAL_TIM_CHANNEL_STATE_READY);
    pHandle->timer.State = HAL_TIM_STATE_READY;

    return eSTATUS_SUCCESS;
}

STATUS_TYPE PWMInitChannel (PWMHandle* pHandle) {

    if (PWM_CHECK_OK (pHandle) == FALSE) {
        return eSTATUS_FAILURE;
    }

    uint32_t timerChannelID      = pHandle->channelID;
    TIM_TypeDef* pTimerRegisters = pHandle->timer.Instance;
    if (timerChannelID == TIM_CHANNEL_1 && IS_TIM_CC1_INSTANCE (pTimerRegisters)) {
        uint32_t tmpccmrx;
        uint32_t tmpccer;
        uint32_t tmpcr2;
        /* Get the TIMx CCER register value */
        tmpccer = pTimerRegisters->CCER;
        /* Disable the Channel 1: Reset the CC1E Bit */
        pTimerRegisters->CCER &= ~TIM_CCER_CC1E;
        /* Get the TIMx CR2 register value */
        tmpcr2 = pTimerRegisters->CR2;
        /* Get the TIMx CCMR1 register value */
        tmpccmrx = pTimerRegisters->CCMR1;
        /* Reset the Output Compare Mode Bits */
        tmpccmrx &= ~TIM_CCMR1_OC1M;
        tmpccmrx &= ~TIM_CCMR1_CC1S;
        /* Select the Output Compare Mode */
        tmpccmrx |= TIM_OCMODE_PWM1;
        /* Reset the Output Polarity level */
        tmpccer &= ~TIM_CCER_CC1P;
        /* Set the Output Compare Polarity */
        tmpccer |= TIM_OCPOLARITY_HIGH;

        if (IS_TIM_CCXN_INSTANCE (pTimerRegisters, TIM_CHANNEL_1)) {
            /* Reset the Output N Polarity level */
            tmpccer &= ~TIM_CCER_CC1NP;
            /* Set the Output N Polarity */
            tmpccer |= TIM_OCNPOLARITY_HIGH;
            /* Reset the Output N State */
            tmpccer &= ~TIM_CCER_CC1NE;
        }

        if (IS_TIM_BREAK_INSTANCE (pTimerRegisters)) {
            /* Reset the Output Compare and Output Compare N IDLE State */
            tmpcr2 &= ~TIM_CR2_OIS1;
            tmpcr2 &= ~TIM_CR2_OIS1N;
            /* Set the Output Idle state */
            tmpcr2 |= TIM_OCIDLESTATE_RESET;
            /* Set the Output N Idle state */
            tmpcr2 |= TIM_OCNIDLESTATE_RESET;
        }

        /* Write to TIMx CR2 */
        pTimerRegisters->CR2 = tmpcr2;
        /* Write to TIMx CCMR1 */
        pTimerRegisters->CCMR1 = tmpccmrx;
        /* Set the Capture Compare Register value */
        pTimerRegisters->CCR1 = 0;
        /* Write to TIMx CCER */
        pTimerRegisters->CCER = tmpccer;
        /* Set the Preload enable bit for channel1 */
        pTimerRegisters->CCMR1 |= TIM_CCMR1_OC1PE;
        /* Configure the Output Fast mode */
        pTimerRegisters->CCMR1 &= ~TIM_CCMR1_OC1FE;
        pTimerRegisters->CCMR1 |= TIM_OCFAST_DISABLE;

    } else if (timerChannelID == TIM_CHANNEL_2 && IS_TIM_CC2_INSTANCE (pTimerRegisters)) {
        uint32_t tmpccmrx;
        uint32_t tmpccer;
        uint32_t tmpcr2;

        /* Get the TIMx CCER register value */
        tmpccer = pTimerRegisters->CCER;
        /* Disable the Channel 2: Reset the CC2E Bit */
        pTimerRegisters->CCER &= ~TIM_CCER_CC2E;
        /* Get the TIMx CR2 register value */
        tmpcr2 = pTimerRegisters->CR2;
        /* Get the TIMx CCMR1 register value */
        tmpccmrx = pTimerRegisters->CCMR1;
        /* Reset the Output Compare mode and Capture/Compare selection Bits */
        tmpccmrx &= ~TIM_CCMR1_OC2M;
        tmpccmrx &= ~TIM_CCMR1_CC2S;
        /* Select the Output Compare Mode */
        tmpccmrx |= (TIM_OCMODE_PWM1 << 8U);
        /* Reset the Output Polarity level */
        tmpccer &= ~TIM_CCER_CC2P;
        /* Set the Output Compare Polarity */
        tmpccer |= (TIM_OCPOLARITY_HIGH << 4U);

        if (IS_TIM_CCXN_INSTANCE (pTimerRegisters, TIM_CHANNEL_2)) {
            /* Reset the Output N Polarity level */
            tmpccer &= ~TIM_CCER_CC2NP;
            /* Set the Output N Polarity */
            tmpccer |= (TIM_OCNPOLARITY_HIGH << 4U);
            /* Reset the Output N State */
            tmpccer &= ~TIM_CCER_CC2NE;
        }

        if (IS_TIM_BREAK_INSTANCE (pTimerRegisters)) {
            /* Reset the Output Compare and Output Compare N IDLE State */
            tmpcr2 &= ~TIM_CR2_OIS2;
            tmpcr2 &= ~TIM_CR2_OIS2N;
            /* Set the Output Idle state */
            tmpcr2 |= (TIM_OCIDLESTATE_RESET << 2U);
            /* Set the Output N Idle state */
            tmpcr2 |= (TIM_OCNIDLESTATE_RESET << 2U);
        }

        /* Write to TIMx CR2 */
        pTimerRegisters->CR2 = tmpcr2;
        /* Write to TIMx CCMR1 */
        pTimerRegisters->CCMR1 = tmpccmrx;
        /* Set the Capture Compare Register value */
        pTimerRegisters->CCR2 = 0U;
        /* Write to TIMx CCER */
        pTimerRegisters->CCER = tmpccer;
        /* Set the Preload enable bit for channel2 */
        pTimerRegisters->CCMR1 |= TIM_CCMR1_OC2PE;
        /* Configure the Output Fast mode */
        pTimerRegisters->CCMR1 &= ~TIM_CCMR1_OC2FE;
        pTimerRegisters->CCMR1 |= TIM_OCFAST_DISABLE << 8U;
    } else {
        LOG_ERROR ("Unsupported timer or timer channel id [0x%X] for PWM initialization", (uint16_t)timerChannelID);
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

STATUS_TYPE PWMInitGPIO (PWMHandle* pHandle) {
    if (PWM_CHECK_OK (pHandle) == FALSE) {
        LOG_ERROR ("Received invalid PWM handle or timer registers pointer");
        return eSTATUS_FAILURE;
    }
    TIM_TypeDef* pTimerRegisters     = pHandle->timer.Instance;
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    if (pTimerRegisters == TIM8) {
        /* Peripheral clock enable */
        // __HAL_RCC_TIM8_CLK_ENABLE ();
        __HAL_RCC_GPIOC_CLK_ENABLE ();
        __HAL_RCC_GPIOJ_CLK_ENABLE ();
        /* TIM8 GPIO Configuration
         * PC6     ------> TIM8_CH1
         * PJ7     ------> TIM8_CH2N
         */
        GPIO_InitStruct.Pin       = GPIO_PIN_6;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init (GPIOC, &GPIO_InitStruct);

        // #define ARD_D6_Pin GPIO_PIN_7
        GPIO_InitStruct.Pin       = GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        // #define ARD_D6_GPIO_Port GPIOJ
        HAL_GPIO_Init (GPIOJ, &GPIO_InitStruct);

    } else if (pTimerRegisters == TIM13) {
        /* Peripheral clock enable */
        // __HAL_RCC_TIM13_CLK_ENABLE ();
        __HAL_RCC_GPIOF_CLK_ENABLE ();
        /* TIM13 GPIO Configuration
         * PF8     ------> TIM13_CH1
         */
        // #define PMOD_14_ARD_D3_Pin       GPIO_PIN_8
        GPIO_InitStruct.Pin       = GPIO_PIN_8;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF9_TIM13;
        // #define PMOD_14_ARD_D3_GPIO_Port GPIOF
        HAL_GPIO_Init (GPIOF, &GPIO_InitStruct);
    } else {
        LOG_ERROR ("Unsupported timer for PWM initialization");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}


STATUS_TYPE PWMInit (PWMConfig config, PWMHandle* pOutHandle) {

    if (pOutHandle == NULL) {
        LOG_ERROR ("PWM output handle pointer is NULL");
        return eSTATUS_FAILURE;
    }

    if (PWM_CHECK_CONF_OK (&config) == FALSE) {
        LOG_ERROR ("Received invalid PWM configuration pointer or timer pointer");
        return eSTATUS_FAILURE;
    }

    memset (pOutHandle, 0, sizeof (PWMHandle));
    PWMHandle pwm                    = { 0 };
    pwm.timer.Instance               = config.base.pTimer;
    pwm.channelID                    = config.base.channelID;
    pwm.timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (config.base.hzPeriod == 0) {
        pwm.timer.Init.Prescaler = 0;
        pwm.timer.Init.Period    = 0;
    } else {
        /* Prescale 64 MHz to 1MHz */
        pwm.timer.Init.Prescaler = (uint16_t)(SystemCoreClock / 1000000U) - 1U;
        /* Scale  1MHz i.e. current PWM Period (ARR) to config.hzPeriod */
        pwm.timer.Init.Period = (uint16_t)(1000000U / config.base.hzPeriod) - 1U;
    }


    STATUS_TYPE status = PWMInitTimBaseConfig (&pwm);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize timer base configuration");
        return status;
    }

    status = PWMInitGPIO (&pwm);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize GPIO for timer");
        return status;
    }

    status = PWMInitChannel (&pwm);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize PWM channel");
        return status;
    }

    *pOutHandle = pwm;
    return eSTATUS_SUCCESS;
}

STATUS_TYPE PWMStart (PWMHandle* pHandle) {
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

STATUS_TYPE PWMWrite (PWMHandle* pHandle, uint32_t usUpTime) {

    if (PWM_CHECK_OK (pHandle) == FALSE) {
        LOG_ERROR ("Received invalid PWM pointer");
        return eSTATUS_FAILURE;
    }

    /* ARR is synonymous with microseconds because the clock is prescaled to 1MHz */
    /* Set ARR and PSC here because sometimes they don't get set properly */
    pHandle->timer.Instance->ARR = pHandle->timer.Init.Period;
    pHandle->timer.Instance->PSC = pHandle->timer.Init.Prescaler;

    // uint32_t usPeriod         = pHandle->arr_ + 1U;
    // float dutyCyclePercentage = PWM_US2DC (usUpTime, usPeriod);
    // uint32_t compareValue =
    // (uint32_t)((dutyCyclePercentage / 100.0F) * (float)(usPeriod));

    /* Debug output to understand what's happening */
    // LOG_INFO("PWM Send - ARR: %u, Period: %u us, Pulse: %u us, Duty: %u%%, CCR: %u",
    //          (uint16_t)(pHandle->pTimerRegisters->ARR),
    //          (uint16_t)usPeriod, (uint16_t)usUpTime,
    //          (uint16_t)dutyCyclePercentage, (uint16_t)compareValue);

    PWM_SET_COMPARE (pHandle, usUpTime);
    return eSTATUS_SUCCESS;
}

STATUS_TYPE
PWMDMAInit (PWM_DMAConfig timConfig, DMAConfig dmaConfig, PWM_DMAHandle* pOutHandle) {

    if (pOutHandle == NULL) {
        LOG_ERROR ("Received nullptr for PWM_DMAHandle");
        return eSTATUS_FAILURE;
    }

    if (PWMDMA_CHECK_CONF_OK (&timConfig) == FALSE) {
        LOG_ERROR ("Received invalid PWM DMA configuration pointer or timer pointer");
        return eSTATUS_FAILURE;
    }

    memset (pOutHandle, 0, sizeof (PWM_DMAHandle));
    PWM_DMAHandle pwm_DMA = { 0 };
    PWMConfig pwmConfig   = { .base = timConfig.base };
    STATUS_TYPE status    = PWMInit (pwmConfig, &pwm_DMA.tim);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize PWM for DMA");
        return status;
    }

    status = DMAInit (dmaConfig, &pwm_DMA.pDMA);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize DMA");
        return status;
    }

    for (uint16_t i = 0; i < timConfig.dmaRegIDXCount; ++i) {
        if (timConfig.dmaRegIDXs[i] >= MAX_DMA_TIM_REGISTERS) {
            LOG_ERROR ("Invalid DMA register index: %u", timConfig.dmaRegIDXs[i]);
            return eSTATUS_FAILURE;
        }
        pwm_DMA.tim.timer.hdma[timConfig.dmaRegIDXs[i]] = pwm_DMA.pDMA;
    }
    pwm_DMA.pDMA->Parent = &pwm_DMA.tim.timer;

    *pOutHandle = pwm_DMA;
    return eSTATUS_SUCCESS;
}