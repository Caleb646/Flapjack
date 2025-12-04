#include "core/core.h"
#include "common.h"
#include "hal.h"
#include "peripheral/gpio.h"
#include <string.h>

float ge_ScaledSystemCoreClock = 0.0F;
// extern uint32_t __SHARED_MEM_START__;
// extern uint32_t __SHARED_MEM_END__;

FJ_STATIC eSTATUS_t Init_HAL_SysTickTimer (uint32_t tickPriority);
FJ_STATIC eSTATUS_t Init_HAL_Timer (TIM_TypeDef* pTimer, uint32_t tickPriority);
FJ_STATIC eSTATUS_t SystemClock_Config (void);
FJ_STATIC eSTATUS_t ZeroSharedMemory (void);
FJ_STATIC void DWT_Init (void);
FJ_STATIC bool SanityCheckTimers (void);

/*
 * FREERTOS is using the SysTick timer so the HAL tick is driven by
 * TIM3 for CM4
 */
void TIM3_IRQHandler (void) {

    // TIM Update event ONLY
    // Clear update event flag
    TIM3->SR &= ~TIM_IT_UPDATE;
    uwTick += (uint32_t)uwTickFreq;
}

/*
 * FREERTOS is using the SysTick timer so the HAL tick is driven by
 * TIM4 for CM7
 */
void TIM4_IRQHandler (void) {

    TIM4->SR &= ~TIM_IT_UPDATE;
    uwTick += (uint32_t)uwTickFreq;
}

// Called by HAL_Init
void HAL_MspInit (void) {

    __HAL_RCC_SYSCFG_CLK_ENABLE ();
    //  PendSV_IRQn interrupt configuration
    HAL_NVIC_SetPriority (PendSV_IRQn, 15, 0);

    if (IS_CM7_ME ()) {
        // Enable SEV irq from CM4
        // CM4_SEV_IRQn interrupt configuration
        HAL_NVIC_SetPriority (CM4_SEV_IRQn, 9, 9);
        HAL_NVIC_EnableIRQ (CM4_SEV_IRQn);
    } else {
        // Enable SEV irq from CM7
        // CM7_SEV_IRQn interrupt configuration
        HAL_NVIC_SetPriority (CM7_SEV_IRQn, 9, 9);
        HAL_NVIC_EnableIRQ (CM7_SEV_IRQn);
    }
}

// Called by HAL_Init
HAL_StatusTypeDef HAL_InitTick (uint32_t tickPriority) {

    if (IS_CM7_ME ()) {
        return (HAL_StatusTypeDef)Init_HAL_Timer (TIM4, tickPriority);
    } else {
        return (HAL_StatusTypeDef)Init_HAL_Timer (TIM3, tickPriority);
    }
}

eSTATUS_t Core_Init (void) {

    if (IS_CM7_ME ()) {
        // Wait until CPU2 boots and enters in stop mode
        while (__HAL_RCC_GET_FLAG (RCC_FLAG_D2CKRDY) != RESET) {
            __NOP ();
        }

        if (HAL_Init () != HAL_OK) {
            return eSTATUS_FAILURE;
        }

        if (FJ_FAIL (SystemClock_Config ())) {
            return eSTATUS_FAILURE;
        }

        RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit = { 0 };
        // select clock source for uart peripherals
        RCC_PeriphClkInit.PeriphClockSelection  = RCC_PERIPHCLK_USART16 | RCC_PERIPHCLK_USART234578;
        RCC_PeriphClkInit.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
        RCC_PeriphClkInit.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
        HAL_RCCEx_PeriphCLKConfig (&RCC_PeriphClkInit);
        // select clock source for spi peripherals
        RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SPI123 | RCC_PERIPHCLK_SPI45; // | RCC_PERIPHCLK_SPI6;
        RCC_PeriphClkInit.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
        RCC_PeriphClkInit.Spi45ClockSelection  = RCC_SPI45CLKSOURCE_PCLK2;
        // RCC_PeriphClkInit.Spi6ClockSelection   = RCC_SPI6CLKSOURCE_D3PCLK1;
        HAL_RCCEx_PeriphCLKConfig (&RCC_PeriphClkInit);
        // select clock source for i2c peripherals
        RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C123 | RCC_PERIPHCLK_I2C4;
        RCC_PeriphClkInit.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
        RCC_PeriphClkInit.I2c4ClockSelection   = RCC_I2C4CLKSOURCE_D3PCLK1;
        HAL_RCCEx_PeriphCLKConfig (&RCC_PeriphClkInit);

        if (FJ_FAIL (ZeroSharedMemory ())) {
            return eSTATUS_FAILURE;
        }

        if (FJ_FAIL (GPIOSystemInit ())) {
            return eSTATUS_FAILURE;
        }

        if (FJ_FAIL (DeviceTree_Init ())) {
            return eSTATUS_FAILURE;
        }

    } else {
        __HAL_RCC_HSEM_CLK_ENABLE ();
        HAL_HSEM_ActivateNotification (__HAL_HSEM_SEMID_TO_MASK (SYS_SEM_ID));
        /*
        Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
        perform system initialization (system clock config, external memory configuration.. )
        */
        HAL_PWREx_ClearPendingEvent ();
        HAL_PWREx_EnterSTOPMode (PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
        __HAL_HSEM_CLEAR_FLAG (__HAL_HSEM_SEMID_TO_MASK (SYS_SEM_ID));
    }

    DWT_Init ();
    // Scale by 10Mhz for fDelayMicroseconds function
    ge_ScaledSystemCoreClock = (float)SystemCoreClock / 10000000.0F;
    if (SanityCheckTimers () == false) {
        return eSTATUS_FAILURE;
    }

    if (SyncInit () != eSTATUS_SUCCESS) {
        return eSTATUS_FAILURE;
    }

    if (LoggerInit () != eSTATUS_SUCCESS) {
        return eSTATUS_FAILURE;
    }

    if (IS_CM7_ME ()) {
        // When system initialization is finished, Cortex-M7
        // will release Cortex-M4 by means of HSEM notification
        __HAL_RCC_HSEM_CLK_ENABLE ();
        SysSem_Take ();
        // Release HSEM in order to notify the CPU2(CM4)
        SysSem_Release ();
        // Wait until CPU2 wakes up from stop mode
        while (__HAL_RCC_GET_FLAG (RCC_FLAG_D2CKRDY) == RESET) {
            __NOP ();
        }
    }

    return eSTATUS_SUCCESS;
}

FJ_UNUSED_FN_DECL FJ_STATIC eSTATUS_t Init_HAL_SysTickTimer (uint32_t tickPriority) {

    FJ_UNUSED (tickPriority);

    return eSTATUS_FAILURE;

    // The below sets up the HAL to use the SysTick ARM core timer, but when FREERTOS is
    // used, FREERTOS takes over the SysTick timer. So the HAL needs to use another seperate timer.

    // // Check uwTickFreq for MisraC 2012 (even if uwTickFreq is a enum type that don't take the
    // value zero) if ((uint32_t)uwTickFreq == 0UL) {
    //     return HAL_ERROR;
    // }

    // // Configure the SysTick to have interrupt in 1ms time basis
    // if (HAL_SYSTICK_Config (SystemCoreClock / (1000UL / (uint32_t)uwTickFreq)) > 0U) {
    //     return HAL_ERROR;
    // }

    // // Configure the SysTick IRQ priority
    // if (TickPriority < (1UL << __NVIC_PRIO_BITS)) {
    //     HAL_NVIC_SetPriority (SysTick_IRQn, TickPriority, 0U);
    //     uwTickPrio = TickPriority;
    // } else {
    //     return HAL_ERROR;
    // }
    // return HAL_OK;
}

FJ_STATIC eSTATUS_t Init_HAL_Timer (TIM_TypeDef* pTimer, uint32_t tickPriority) {

    RCC_ClkInitTypeDef clkconfig;
    uint32_t uwTimclock, uwAPB1Prescaler;
    uint32_t uwPrescalerValue;
    uint32_t pFLatency;

    if (tickPriority < (1UL << __NVIC_PRIO_BITS)) {

        if (pTimer == TIM3) {

            HAL_NVIC_SetPriority (TIM3_IRQn, tickPriority, 0);
            HAL_NVIC_EnableIRQ (TIM3_IRQn);
            __HAL_RCC_TIM3_CLK_ENABLE ();
        } else if (pTimer == TIM4) {

            HAL_NVIC_SetPriority (TIM4_IRQn, tickPriority, 0);
            HAL_NVIC_EnableIRQ (TIM4_IRQn);
            uwTickPrio = tickPriority;
            __HAL_RCC_TIM4_CLK_ENABLE ();
        }

    } else {
        return eSTATUS_FAILURE;
    }

    /* Get clock configuration */
    HAL_RCC_GetClockConfig (&clkconfig, &pFLatency);
    /* Get APB1 prescaler */
    uwAPB1Prescaler = clkconfig.APB1CLKDivider;
    /* Compute TIM clock */
    if (uwAPB1Prescaler == RCC_HCLK_DIV1) {
        uwTimclock = HAL_RCC_GetPCLK1Freq ();
    } else {
        uwTimclock = 2UL * HAL_RCC_GetPCLK1Freq ();
    }

    /* Compute the prescaler value to have TIM4 counter clock equal to 1MHz */
    uwPrescalerValue = (uint32_t)((uwTimclock / 1000000U) - 1U);

    TIM_HandleTypeDef htim = { 0 };
    htim.Instance          = pTimer;

    /* Initialize TIMx peripheral as follow:
     * Period = [(TIM4CLK/1000) - 1]. to have a (1/1000) s time base.
     * Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
     * ClockDivision = 0
     * Counter direction = Up
     */
    htim.Init.Period        = (1000000U / 1000U) - 1U;
    htim.Init.Prescaler     = uwPrescalerValue;
    htim.Init.ClockDivision = 0;
    htim.Init.CounterMode   = TIM_COUNTERMODE_UP;

    if (HAL_TIM_Base_Init (&htim) == HAL_OK) {
        /* Start the TIM time Base generation in interrupt mode */
        return HAL_TIM_Base_Start_IT (&htim);
    }

    /* Return function status */
    return eSTATUS_FAILURE;
}

FJ_STATIC eSTATUS_t SystemClock_Config (void) {

    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Supply configuration update enable
     */
    // TODO: this is fine for DEV board but for custom board SMPS is NOT setup
    // Default power supply on reset should be LDO

    HAL_PWREx_ConfigSupply (PWR_DIRECT_SMPS_SUPPLY);
    // HAL_PWREx_ConfigSupply (PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG (PWR_REGULATOR_VOLTAGE_SCALE1);

    while (!__HAL_PWR_GET_FLAG (PWR_FLAG_VOSRDY)) {
    }

    /** Initializes the RCC Oscillators according to the specified
     * parameters in the RCC_OscInitTypeDef structure.
     */
    // TODO: Custom board will NOT have HSE crystal
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState            = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.LSIState            = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM            = 5;
    RCC_OscInitStruct.PLL.PLLN            = 48;
    RCC_OscInitStruct.PLL.PLLP            = 2;
    RCC_OscInitStruct.PLL.PLLQ            = 5;
    RCC_OscInitStruct.PLL.PLLR            = 2;
    RCC_OscInitStruct.PLL.PLLRGE          = RCC_PLL1VCIRANGE_2;
    RCC_OscInitStruct.PLL.PLLVCOSEL       = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN        = 0;
    if (HAL_RCC_OscConfig (&RCC_OscInitStruct) != HAL_OK) {
        return eSTATUS_FAILURE;
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

    if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        return eSTATUS_FAILURE;
    }
    HAL_RCC_MCOConfig (RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
    return eSTATUS_SUCCESS;
}

FJ_STATIC eSTATUS_t ZeroSharedMemory (void) {

    memset (&__SHARED_MEM_START__, 0, &__SHARED_MEM_END__ - &__SHARED_MEM_START__);
    return eSTATUS_SUCCESS;
}

FJ_STATIC void DWT_Init (void) {

#ifndef UNIT_TEST
    // Enable core debug access and trace unit
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // Enable DWT cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;

#endif // UNIT_TEST
}

FJ_STATIC bool SanityCheckTimers (void) {

    uint32_t const msDelay = 10;
    uint32_t tempStart     = GetMilliseconds ();
    HAL_Delay (msDelay);
    uint32_t tempEnd = GetMilliseconds ();

    if (tempEnd == tempStart || tempEnd < tempStart) {
        return false;
    }

    if ((tempEnd - tempStart) < msDelay - 2U || (tempEnd - tempStart) > msDelay + 2U) {
        return false;
    }

    uint32_t const usDelay = msDelay * 1000U;
    tempStart              = GetMicroseconds ();
    DelayMicroseconds (usDelay);
    tempEnd = GetMicroseconds ();

    if (tempEnd == tempStart || tempEnd < tempStart) {
        return false;
    }

    if ((tempEnd - tempStart) < usDelay - (2U * 1000U) || (tempEnd - tempStart) > usDelay + (2U * 1000U)) {
        return false;
    }

    return true;
}