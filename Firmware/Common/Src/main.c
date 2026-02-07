#include <stdio.h>

#include "control.h"
#include "fcstate.h"
#include "hal.h"
#include "scheduler.h"

#include "core/core.h"

#include "conf/conf.h"

#include "device/device.h"
#include "device/imu/imu.h"
#include "device/mag/mag.h"

#include "mc/actuators.h"
#include "mc/filter.h"
#include "mc/mc.h"
#include "mc/pid.h"

#include "peripheral/bus/bus.h"
#include "peripheral/dma.h"
#include "peripheral/gpio.h"


#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U)
#endif

#ifdef CORE_CM7

void SysTick_Handler (void) {

    HAL_IncTick ();
}

void SystemClock_Config (void);

int main (void) {

    HAL_Init ();
    HAL_NVIC_EnableIRQ (SysTick_IRQn);

    SystemClock_Config ();
    /*HW semaphore Clock enable*/
    __HAL_RCC_HSEM_CLK_ENABLE ();
    /* Take HSEM */
    HAL_HSEM_FastTake (HSEM_ID_0);
    /* Release HSEM in order to notify the CPU2(CM4) */
    HAL_HSEM_Release (HSEM_ID_0, 0);
    /* Wait until CPU2 wakes up from stop mode */
    while (__HAL_RCC_GET_FLAG (RCC_FLAG_D2CKRDY) == RESET) {
    }

    __HAL_RCC_SYSCFG_CLK_ENABLE ();
    // enable CM4_SEV_IRQn so CM4 can send SEV to CM7
    HAL_NVIC_SetPriority (CM4_SEV_IRQn, 9, 9);
    HAL_NVIC_EnableIRQ (CM4_SEV_IRQn);

    if (Core_Init () != eSTATUS_SUCCESS) {
        CriticalErrorHandler ();
    }

    if (BOARD_CONF_INIT () != eSTATUS_SUCCESS) {
        CriticalErrorHandler ();
    }

    if (Device_InitAll (BoardConfGet ()) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to init device module");
        CriticalErrorHandler ();
    }

    if (MC_InitAll () != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to init motion control module");
        CriticalErrorHandler ();
    }
    // TODO: Motors and Servos need to be started somewhere

    Delay (250);

    LOG_INFO ("Starting scheduler");
    Scheduler_Main (CM7_IDX, FJ_LOOP_UPDATE_RATE_HZ);
}

void SystemClock_Config (void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

#if defined(USE_PWR_LDO_SUPPLY) && defined(USE_PWR_DIRECT_SMPS_SUPPLY)
#error "Only one of USE_PWR_LDO_SUPPLY or USE_PWR_DIRECT_SMPS_SUPPLY can be defined"
#endif /* USE_PWR_LDO_SUPPLY && USE_PWR_DIRECT_SMPS_SUPPLY */

#if defined(USE_PWR_DIRECT_SMPS_SUPPLY) && defined(SMPS)
    HAL_PWREx_ConfigSupply (PWR_DIRECT_SMPS_SUPPLY);
#endif /* USE_PWR_DIRECT_SMPS_SUPPLY && SMPS */

#if defined(USE_PWR_LDO_SUPPLY)
    HAL_PWREx_ConfigSupply (PWR_LDO_SUPPLY);
#endif /* USE_PWR_LDO_SUPPLY */

    /* Configure the main internal regulator output voltage */
    __HAL_PWR_VOLTAGESCALING_CONFIG (PWR_REGULATOR_VOLTAGE_SCALE1);

    while (!__HAL_PWR_GET_FLAG (PWR_FLAG_VOSRDY)) {
    }

    /** Initializes the RCC Oscillators according to the specified
     * parameters in the RCC_OscInitTypeDef structure.
     */
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
        CriticalErrorHandler ();
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
        CriticalErrorHandler ();
    }
}

#endif // CORE_CM7

#ifdef CORE_CM4

int main (void) {
    /*HW semaphore Clock enable*/
    __HAL_RCC_HSEM_CLK_ENABLE ();
    /* Activate HSEM notification for Cortex-M4*/
    HAL_HSEM_ActivateNotification (__HAL_HSEM_SEMID_TO_MASK (HSEM_ID_0));

    /* Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 */
    HAL_PWREx_ClearPendingEvent ();
    HAL_PWREx_EnterSTOPMode (PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);

    /* Clear HSEM flag */
    __HAL_HSEM_CLEAR_FLAG (__HAL_HSEM_SEMID_TO_MASK (HSEM_ID_0));

    HAL_Init ();
    HAL_NVIC_EnableIRQ (SysTick_IRQn);

    // enable CM4_SEV_IRQn so CM7 can send SEV to CM4
    HAL_NVIC_SetPriority (CM7_SEV_IRQn, 9, 9);
    HAL_NVIC_EnableIRQ (CM7_SEV_IRQn);

    if (Core_Init () != eSTATUS_SUCCESS) {
        CriticalErrorHandler ();
    }

    Delay (250);

    LOG_INFO ("Starting scheduler");
    Scheduler_Main (CM4_IDX, FJ_LOOP_UPDATE_RATE_HZ);
}

#endif // CORE_CM4