#include <stdio.h>

#include "hal.h"
#include "control.h"
#include "fcstate.h"

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

void SystemClock_Config (void);

void TaskMotionControlUpdate (void) {

    uint32_t msLastUpdate    = GetMilliseconds ();
    uint32_t const msLogStep = MS_PER_LOG_DATA_UPDATE;
    LOG_INFO ("Motion control update task started");

    if (FJ_LOOP_UPDATE_RATE_HZ > 1000U || FJ_LOOP_UPDATE_RATE_HZ < 0U) {
        LOG_ERROR ("Sensor update rate invalid");
    }

    // TODO: TEMPORARY. should be done after receiving a start command
    if (Device_StartAll () != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start Device module");
    }

    if (MC_StartAll () != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start Motion Control module");
    }
    FC_SET_RUNNING_OP_STATE ();

    eSTATUS_t status      = eSTATUS_SUCCESS;
    FCState_t fcState     = { 0 };
    Vec3f currentAttitude = { 0.0F };
    Vec3f targetAttitude  = { 0.0F };
    Vec3f maxAttitude     = { 0.0F };
    Vec3f pidAttitude     = { 0.0F };
    float targetThrottle  = 0.0F;
    float dt              = 0.0F;
    Vec3f accel           = { 0.0F };
    Vec3f gyro            = { 0.0F };
    Vec3f mag             = { 0.0F };
    Vec3f* pMagData       = NULL;

    while (true) {

        fcState = FCState_GetCopyOfActiveState ();
        if (fcState.opState != eOP_STATE_RUNNING) {
            /*
             * Update msLastUpdate so dt does not get too large.
             */
            msLastUpdate = GetMilliseconds ();
            // Limit state checks to 1000Hz
            Delay (1);
            continue;
        }
        /*
         * TODO: notifications from IMU started breaking FreeRTOS.
         * I am hitting this assert sometimes: configASSERT( listLIST_ITEM_CONTAINER( &( pxTCB->xEventListItem ) ) == NULL in tasks.c
         */
        // ulTaskNotifyTake (pdTRUE, pdMS_TO_TICKS (1000));
        vIMU_t* pIMUDev    = IMU_GetMutableActiveDevice ();
        vMag_t* pMagDev    = Mag_GetMutableActiveDevice ();
        vFilter_t* pFilter = Filter_GetMutableActiveFilter ();
        vPID_t* pPID       = PID_GetMutableActivePID ();
        status             = eSTATUS_SUCCESS;
        currentAttitude    = fcState.currentAttitude;
        targetAttitude     = fcState.targetAttitude;
        maxAttitude        = fcState.maxAttitude;
        targetThrottle     = fcState.targetThrottle;
        dt                 = ((float)GetMilliseconds () - (float)msLastUpdate) / 1000.0F;
        // set to NULL each iteration
        pMagData = NULL;

        if (STATUS_FAIL (IMU_Update (pIMUDev, false, &accel, &gyro))) {
            LOG_ERROR ("Failed to get IMU data");
            continue;
        }

        // NOTE: Okay if magnetometer data is not available
        if (STATUS_OK (Mag_Update (pMagDev, false, &mag))) {
            pMagData = &mag;
        }

        status = Filter_Update (pFilter, &accel, &gyro, pMagData, dt, &currentAttitude);
        if (STATUS_FAIL (status)) {
            LOG_ERROR ("Failed to filter IMU data with Madgwick filter");
            continue;
        }


        status = PID_Update (pPID, &currentAttitude, &targetAttitude, &maxAttitude, dt, &pidAttitude);
        if (STATUS_FAIL (status)) {
            LOG_ERROR ("Failed to update PID attitude");
            continue;
        }

        status = Actuators_Update (pidAttitude, targetThrottle);
        if (STATUS_FAIL (status)) {
            LOG_ERROR ("Failed to write actuators");
            continue;
        }

        if (FC_SET_CURRENT_ATTITUDE (currentAttitude) == false) {
            LOG_ERROR ("Failed to set current attitude in FCState");
            continue;
        }

        if ((GetMilliseconds () - msLastUpdate) >= msLogStep) {

            Vec3f a   = accel;
            Vec3f g   = gyro;
            Vec3f ca  = currentAttitude;
            Vec3f pid = pidAttitude;
            pid.roll *= maxAttitude.roll;
            pid.pitch *= maxAttitude.pitch;
            pid.yaw *= maxAttitude.yaw;

            // portENTER_CRITICAL ();
            LOG_DATA_IMU_DATA (a, g);
            LOG_DATA_CURRENT_ATTITUDE (ca);
            LOG_DATA_CURRENT_PID_ATTITUDE (pid);
            ActuatorsLogData ();
            // portEXIT_CRITICAL ();
        }

        msLastUpdate = GetMilliseconds ();
        // Limit loop to sensor update rate
        vTaskDelay (pdMS_TO_TICKS (1000U / FJ_LOOP_UPDATE_RATE_HZ));
    }
}

int main (void) {

    HAL_Init ();
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

    if (Core_Init () != eSTATUS_SUCCESS) {
        CriticalErrorHandler ();
    }

    if (Control_Init () != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to init control module");
        CriticalErrorHandler ();
    }

    Delay (1000);

    if (ControlStart () != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start control module");
        CriticalErrorHandler ();
    }

    uint32_t startTime = HAL_GetTick ();
    uint32_t logStep   = 5000;

    while (1) {
        SyncProcessTasks ();
        ControlProcess_RawCmds ();

        if ((HAL_GetTick () - startTime) >= logStep) {
            startTime = HAL_GetTick ();
            LOG_INFO ("CM7 main loop running");
        }
    }
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

    __HAL_RCC_SYSCFG_CLK_ENABLE ();
    __HAL_PWR_VOLTAGESCALING_CONFIG (PWR_REGULATOR_VOLTAGE_SCALE0);

    while (!__HAL_PWR_GET_FLAG (PWR_FLAG_VOSRDY)) {
    }

    /* Configure HSE and PLL */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 4;
    RCC_OscInitStruct.PLL.PLLN       = 400;
    RCC_OscInitStruct.PLL.PLLP       = 2;
    RCC_OscInitStruct.PLL.PLLQ       = 4;
    RCC_OscInitStruct.PLL.PLLR       = 2;
    RCC_OscInitStruct.PLL.PLLRGE     = RCC_PLL1VCIRANGE_1;
    RCC_OscInitStruct.PLL.PLLVCOSEL  = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN   = 0;
    if (HAL_RCC_OscConfig (&RCC_OscInitStruct) != HAL_OK) {
        CriticalErrorHandler ();
    }

    /* Configure system clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
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

    if (Core_Init () != eSTATUS_SUCCESS) {
        CriticalErrorHandler ();
    }

    if (Control_Init () != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to init control module");
        CriticalErrorHandler ();
    }

    Delay (1000);

    if (ControlStart () != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start control module");
        CriticalErrorHandler ();
    }

    uint32_t startTime = HAL_GetTick ();
    uint32_t logStep   = 5000;

    while (1) {
        SyncProcessTasks ();
        ControlProcess_RawCmds ();

        if ((HAL_GetTick () - startTime) >= logStep) {
            startTime = HAL_GetTick ();
            LOG_INFO ("CM4 main loop running");
        }
    }
}

#endif // CORE_CM4