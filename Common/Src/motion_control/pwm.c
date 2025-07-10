#include "motion_control/pwm.h"
#include "common.h"
#include "hal.h"
#include "log.h"

STATUS_TYPE PWMInitTimBaseConfig (PWMHandle* pHandle) {

    if (pHandle == NULL || pHandle->pTimerRegisters == NULL) {
        return eSTATUS_FAILURE;
    }

    TIM_TypeDef* pTimerRegisters = pHandle->pTimerRegisters;

    uint32_t tmpcr1 = pTimerRegisters->CR1;
    /* Set the counter mode */
    if (IS_TIM_COUNTER_MODE_SELECT_INSTANCE (pTimerRegisters)) {
        tmpcr1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);
        tmpcr1 |= TIM_COUNTERMODE_UP;
    }

    /* Set the clock division */
    if (IS_TIM_CLOCK_DIVISION_INSTANCE (pTimerRegisters)) {
        tmpcr1 &= ~TIM_CR1_CKD;
        tmpcr1 |= (uint32_t)TIM_CLOCKDIVISION_DIV1;
    }

    /* Set the auto-reload preload */
    tmpcr1 &= ~TIM_CR1_ARPE;
    tmpcr1 |= TIM_AUTORELOAD_PRELOAD_ENABLE;
    /* Set the Autoreload value */
    pTimerRegisters->ARR = pHandle->period; // 65535U;
    /* Set the Prescaler value */
    pTimerRegisters->PSC = pHandle->prescaler;

    if (IS_TIM_REPETITION_COUNTER_INSTANCE (pTimerRegisters)) {
        /* Set the Repetition Counter value */
        pTimerRegisters->RCR = 0U;
    }

    /* Disable Update Event (UEV) with Update Generation (UG)
       by changing Update Request Source (URS) to avoid Update flag (UIF) */
    pTimerRegisters->CR1 |= TIM_CR1_URS;
    /* Generate an update event to reload the Prescaler
       and the repetition counter (only for advanced timer) value immediately */
    pTimerRegisters->EGR = TIM_EGR_UG;
    pTimerRegisters->CR1 = tmpcr1;

    return eSTATUS_SUCCESS;
}

STATUS_TYPE PWMInitChannel (PWMHandle* pHandle) {

    if (pHandle == NULL || pHandle->pTimerRegisters == NULL) {
        return eSTATUS_FAILURE;
    }

    uint32_t timerChannelID      = pHandle->timerChannelID;
    TIM_TypeDef* pTimerRegisters = pHandle->pTimerRegisters;
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
    if (pHandle == NULL || pHandle->pTimerRegisters == NULL) {
        LOG_ERROR ("Received invalid PWM handle or timer registers pointer");
        return eSTATUS_FAILURE;
    }
    TIM_TypeDef* pTimerRegisters     = pHandle->pTimerRegisters;
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    if (pTimerRegisters == TIM8) {
        /* Peripheral clock enable */
        __HAL_RCC_TIM8_CLK_ENABLE ();
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
        __HAL_RCC_TIM13_CLK_ENABLE ();
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


STATUS_TYPE PWMInit (PWMHandle* pHandle) {

    if (pHandle == NULL || pHandle->pTimerRegisters == NULL) {
        LOG_ERROR ("PWM handle pointer or timer registers pointer is NULL");
        return eSTATUS_FAILURE;
    }

    STATUS_TYPE status = PWMInitTimBaseConfig (pHandle);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize timer base configuration");
        return status;
    }

    status = PWMInitGPIO (pHandle);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize GPIO for timer");
        return status;
    }

    status = PWMInitChannel (pHandle);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize PWM channel");
        return status;
    }

    return eSTATUS_SUCCESS;
}

STATUS_TYPE PWMStart (PWMHandle* pHandle) {
    if (pHandle == NULL || pHandle->pTimerRegisters == NULL) {
        LOG_ERROR ("Received invalid PWM pointer");
        return eSTATUS_FAILURE;
    }

    /* 0x1FU = 31 bits max shift */
    uint32_t tmp = TIM_CCER_CC1E << (pHandle->timerChannelID & 0x1FU);
    /* Reset the CCxE Bit */
    pHandle->pTimerRegisters->CCER &= ~tmp;
    /* Set or reset the CCxE Bit */
    /* 0x1FU = 31 bits max shift */
    pHandle->pTimerRegisters->CCER |=
    (uint32_t)(TIM_CCx_ENABLE << (pHandle->timerChannelID & 0x1FU));

    if (IS_TIM_BREAK_INSTANCE (pHandle->pTimerRegisters) != FALSE) {
        /* Enable the Main Output */
        pHandle->pTimerRegisters->BDTR |= TIM_BDTR_MOE;
    }
    pHandle->pTimerRegisters->CR1 |= TIM_CR1_CEN;
    return eSTATUS_SUCCESS;
}

STATUS_TYPE PWMSend (PWMHandle* pHandle, uint32_t usTargetDutyCycle) {

    // 1,000,000 Hz / ARR = 50Hz --> ARR = 20,000
    // 1.5ms duty cycle = td (target duty cycle)
    // td / period = percentage
    // CCR / ARR = percentage
    // CCR = ARR * (1500us / 20000us)

    // 1,000,000 Hz / ARR = 2000Hz --> ARR = 500
    // 250us duty cycle = td (target duty cycle)
    // td / period = percentage
    // CCR / ARR = percentage
    // CCR = ARR * (250us / 500us)

    // *pCCR = (uint16_t)pPWM->usTargetDutyCycle;
    if (PWM_CHECK_OK (pHandle) != TRUE) {
        LOG_ERROR ("Received invalid PWM pointer");
        return eSTATUS_FAILURE;
    }
    PWM_SET_COMPARE (pHandle, usTargetDutyCycle);
    return eSTATUS_SUCCESS;
}