/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#include "main.h"
// #include "cmsis_os.h"

#include <stdio.h>

// #include "FreeRTOS.h"
// #include "semphr.h"
// #include "task.h"
// #include "timers.h"

#include "control.h"
#include "core/core.h"
#include "peripheral/bus/bus.h"


#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

void TaskMainLoop (void* pvParameters) {

    FJ_UNUSED (pvParameters);

    uint32_t startTime = xTaskGetTickCount ();
    uint32_t logStep   = 5000;

    if (ControlStart () != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start control module");
        CriticalErrorHandler ();
    }

    while (1) {
        SyncProcessTasks ();
        ControlProcess_RawCmds ();
        if ((xTaskGetTickCount () - startTime) >= logStep) {
            startTime = xTaskGetTickCount ();
            LOG_INFO ("Main loop running");
        }
    }
}

int main (void) {

    /*HW semaphore Clock enable*/
    __HAL_RCC_HSEM_CLK_ENABLE ();
    /* Activate HSEM notification for Cortex-M4*/
    HAL_HSEM_ActivateNotification (__HAL_HSEM_SEMID_TO_MASK (HSEM_ID_0));
    /*
    Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
    perform system initialization (system clock config, external memory configuration.. )
    */
    HAL_PWREx_ClearPendingEvent ();
    HAL_PWREx_EnterSTOPMode (PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
    /* Clear HSEM flag */
    __HAL_HSEM_CLEAR_FLAG (__HAL_HSEM_SEMID_TO_MASK (HSEM_ID_0));

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init ();

    if (Core_Init () != eSTATUS_SUCCESS) {
        CriticalErrorHandler ();
    }

    if (Control_Init () != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to init control module");
        CriticalErrorHandler ();
    }
    Delay (1000);

    uint16_t taskPriority = 1;
    BaseType_t taskStatus =
    xTaskCreate (TaskMainLoop, "Motion Control Update Task", configMINIMAL_STACK_SIZE, NULL, taskPriority, NULL);

    if (taskStatus != pdPASS) {
        LOG_ERROR ("Failed to create motion control update task");
    }

    vTaskStartScheduler ();

    while (1) {
    }
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM3 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to
 * increment a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef* htim) {

    if (htim->Instance == TIM3) {
        HAL_IncTick ();
    }
}
