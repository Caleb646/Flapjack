/* USER CODE BEGIN Header */
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include "common.h"
#include "control.h"
#include "dma.h"
#include "log.h"
#include "mc/actuators.h"
#include "mc/filter.h"
#include "periphs/gpio.h"
#include "sensors/imu/imu.h"
#include "sync.h"

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

SPI_HandleTypeDef hspi2;

void SystemClock_Config (void);
static void MX_GPIO_Init (void);
static void MX_SPI2_Init (void);

// NOLINTBEGIN
IMU gIMU                                     = { 0 };
FilterMadgwickContext gFilterMadgwickContext = { 0 };
PIDContext gPIDAngleContext                  = { 0 };
TaskHandle_t gpTaskMotionControlUpdate       = { 0 };
Vec3f gTargetAttitude                        = { 0.0F };
float gTargetThrottle                        = 0.25F; // 25% throttle
// NOLINTEND

/**
 * @brief This function handles EXTI line[9:5] interrupts. Call chain is:
 * EXTI9_5_IRQHandler -> HAL_GPIO_EXTI_IRQHandler -> HAL_GPIO_EXTI_Callback
 */
void EXTI9_5_IRQHandler (void) {
    HAL_GPIO_EXTI_IRQHandler (IMU_INT_GPIO_Pin);
}

void HAL_GPIO_EXTI_Callback (uint16_t gpioPin) {
    if (gpioPin == IMU_INT_GPIO_Pin) {
        eSTATUS_t status = IMU2CPUInterruptHandler (&gIMU);
        if (status == eSTATUS_SUCCESS) {
            if (gpTaskMotionControlUpdate != NULL) {
                xTaskNotifyFromISR (gpTaskMotionControlUpdate, 0, eSetBits, NULL);
            }
        } else {
            gIMU.status = status;
        }
    }
}

void FCEnterRunningState (FCState curState, eREQUESTED_STATE_t requestedState) {

    eOP_STATE_t curOpState  = curState.opState;
    eOP_STATE_t nextOpState = eOP_STATE_RUNNING;
    if (curOpState == eOP_STATE_STOPPED) {
        if (requestedState == eREQUESTED_STATE_START) {
            LOG_INFO ("Starting actuators and imu");
            if (ActuatorsStart () != eSTATUS_SUCCESS || IMUStart (&gIMU) != eSTATUS_SUCCESS) {
                LOG_ERROR ("Failed to start");
                nextOpState = eOP_STATE_ERROR;
            }
        }
    }

    curState.opState = nextOpState;
    if (ControlUpdateFCState (&curState) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to update fc state");
    }
    if (nextOpState == eOP_STATE_RUNNING) {
        LOG_INFO ("FC in running state");
    }
}

void FCEnterStoppedState (FCState curState, eREQUESTED_STATE_t requestedState) {

    eOP_STATE_t curOpState  = curState.opState;
    eOP_STATE_t nextOpState = eOP_STATE_STOPPED;
    if (curOpState == eOP_STATE_RUNNING) {
        if (requestedState == eREQUESTED_STATE_STOP) {
            if (IMUStop (&gIMU) != eSTATUS_SUCCESS || ActuatorsStop () != eSTATUS_SUCCESS) {
                LOG_ERROR ("Failed to stop");
                nextOpState = eOP_STATE_ERROR;
            }
        }
    }

    curState.opState = nextOpState;
    if (ControlUpdateFCState (&curState) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to update fc state");
    }

    if (nextOpState == eOP_STATE_STOPPED) {
        LOG_INFO ("FC in stopped state");
    }
}

void ProcessOPStateChange (FCState curState, eREQUESTED_STATE_t requestedState) {

    if (curState.opState == requestedState) {
        LOG_INFO ("Already in state: %d", curState.opState);
        return;
    }

    if (requestedState == eREQUESTED_STATE_START) {
        FCEnterRunningState (curState, requestedState);
        return;
    }
    if (requestedState == eREQUESTED_STATE_STOP) {
        FCEnterStoppedState (curState, requestedState);
        return;
    }
}

void TaskMainLoop (void* pvParameters) {
    uint32_t startTime = xTaskGetTickCount ();
    uint32_t logStep   = 5000;
    LOG_INFO ("Main loop started");

    if (ControlStart (NULL) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start control module");
        CriticalErrorHandler ();
    }
    while (1) {
        EmptyCommand cmd = { 0 };
        if (ControlGetNewCmd (&cmd) == TRUE) {
            switch (cmd.header.commandType) {
            case eCOMMAND_TYPE_EMPTY:
                LOG_ERROR ("Received empty command");
                break;
            case eCOMMAND_TYPE_CHANGE_OP_STATE:
                ChangeOpStateCmd* opCmd = (ChangeOpStateCmd*)&cmd;
                ProcessOPStateChange (ControlGetCopyFCState (), opCmd->requestedState);
                break;
            case eCOMMAND_TYPE_CHANGE_FLIGHT_MODE:
                // ChangeFlightModeCmd* flightModeCmd = (ChangeFlightModeCmd*)&cmd;
                // ProcessFlightModeChange (ControlGetCopyFCState (), flightModeCmd->requestedMode);
                break;
            case eCOMMAND_TYPE_CHANGE_VELOCITY:
                // ChangeVelocityCmd* velocityCmd = (ChangeVelocityCmd*)&cmd;
                // ProcessVelocityChange (ControlGetCopyFCState (), velocityCmd->requestedVelocity);
                break;
            case eCOMMAND_TYPE_CHANGE_PID:
                // ChangePIDCmd* pidCmd = (ChangePIDCmd*)&cmd;
                // ProcessPIDChange (ControlGetCopyFCState (), pidCmd->pidType, pidCmd->pidValue);
                break;
            default:
                LOG_ERROR ("Unknown command type: %d", cmd.header.commandType);
            }
        }
        // Aim for 500Hz
        vTaskDelay (pdMS_TO_TICKS (2));
    }
}

void TaskMotionControlUpdate (void* pvParameters) {

    float msStartTime        = (float)xTaskGetTickCount ();
    uint32_t msLogStart      = xTaskGetTickCount ();
    uint32_t const msLogStep = 500;
    Vec3f currentAttitude    = { 0.0F };
    Vec3f maxAttitude = { .roll = 45.0F, .pitch = 45.0F, .yaw = 180.0F };
    LOG_INFO ("Motion control update task started");

    while (1) {
        if (ControlGetOpState () != eOP_STATE_RUNNING) {
            // Limit state checks to 1000Hz
            vTaskDelay (pdMS_TO_TICKS (1));
            continue;
        }
        ulTaskNotifyTake (pdTRUE, pdMS_TO_TICKS (1000));

        eSTATUS_t status = eSTATUS_SUCCESS;
        Vec3f accel      = { 0.0F };
        Vec3f gyro       = { 0.0F };
        if (IMUProcessUpdatefromINT (&gIMU, &accel, &gyro) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to process IMU data from interrupt");
            continue;
        }

        float msCurrentTime = (float)xTaskGetTickCount ();
        // Convert milliseconds to seconds
        float dt    = (msCurrentTime - msStartTime) / 1000.0F;
        msStartTime = msCurrentTime;

        if (dt <= 0.0F) {
            continue;
        }

        status =
        FilterMadgwick6DOF (&gFilterMadgwickContext, &accel, &gyro, dt, &currentAttitude);
        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to filter IMU data with Madgwick filter");
            continue;
        }

        Vec3f pidAttitude = { 0.0F };
        status            = PIDUpdateAttitude (
        &gPIDAngleContext, currentAttitude, gTargetAttitude, maxAttitude, dt, &pidAttitude);
        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to update PID attitude");
            continue;
        }

        status = ActuatorsWrite (pidAttitude, gTargetThrottle);
        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to write actuators");
            continue;
        }

        if ((xTaskGetTickCount () - msLogStart) >= msLogStep) {
            msLogStart = xTaskGetTickCount ();

            Vec3f a  = accel;
            Vec3f g  = gyro;
            Vec3f ca = currentAttitude;

            LOG_DATA_IMU_DATA (a, g);
            LOG_DATA_CURRENT_ATTITUDE (ca);
            ActuatorsLogData ();
        }
    }
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main (void) {

    /* Wait until CPU2 boots and enters in stop mode */
    while (__HAL_RCC_GET_FLAG (RCC_FLAG_D2CKRDY) != RESET) {
        asm volatile ("NOP");
    }

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init ();
    SystemClock_Config ();
    MX_GPIO_Init ();
    MX_SPI2_Init ();

    /* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of HSEM notification */

    /*HW semaphore Clock enable*/
    __HAL_RCC_HSEM_CLK_ENABLE ();
    // /*Take HSEM */
    HAL_HSEM_FastTake (HSEM_ID_0);
    // /*Release HSEM in order to notify the CPU2(CM4)*/
    HAL_HSEM_Release (HSEM_ID_0, 0);
    // /* wait until CPU2 wakes up from stop mode */
    while (__HAL_RCC_GET_FLAG (RCC_FLAG_D2CKRDY) == RESET) {
        asm volatile ("NOP");
    }

    if (SyncInit () != eSTATUS_SUCCESS) {
        CriticalErrorHandler ();
    }

    if (LoggerInit () != eSTATUS_SUCCESS) {
        CriticalErrorHandler ();
    }

    if (ControlInit () != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to init control module");
        CriticalErrorHandler ();
    }
    // Wait for CM4 to initialize UART
    HAL_Delay (1000);

    if (DMASystemInit () != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to init DMA system");
    }

    eSTATUS_t status = eSTATUS_SUCCESS;
    /*
     * Init IMU
     */
    {
        IMUAccConf aconf           = { 0 };
        aconf.odr                  = eIMU_ACC_ODR_200;
        aconf.range                = eIMU_ACC_RANGE_2G;
        aconf.avg                  = eIMU_ACC_AVG_16;
        aconf.bw                   = eIMU_ACC_BW_HALF;
        aconf.mode                 = eIMU_ACC_MODE_HIGH_PERF;
        IMUGyroConf gconf          = { 0 };
        gconf.odr                  = eIMU_GYRO_ODR_200;
        gconf.range                = eIMU_GYRO_RANGE_250;
        gconf.avg                  = eIMU_GYRO_AVG_16;
        gconf.bw                   = eIMU_GYRO_BW_HALF;
        gconf.mode                 = eIMU_GYRO_MODE_HIGH_PERF;
        IMUAxesRemapConf axesRemap = { 0 };
        axesRemap.remap            = eIMU_AXES_REMAP_YXZ;
        axesRemap.xDir             = eIMU_AXES_DIR_INVERTED;
        axesRemap.yDir             = eIMU_AXES_DIR_INVERTED;
        axesRemap.zDir             = eIMU_AXES_DIR_DEFAULT;
        if (IMUInit (&gIMU, &hspi2, aconf, gconf, &axesRemap) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to init IMU");
        }
    }

    PID_INIT (gPIDAngleContext);

    /* With an ODR of 100 Hz on the IMU 1000 iterations will take 10 seconds */
    /* With an ODR of 200 Hz on the IMU 1000 iterations will take 5 seconds */
    /*
     * NOTE: During filter warmup the IMU is polled and interrupts are disabled. So IMUStart doesnt need to be called
     */
    status = FilterMadgwickWarmUp (500U, &gIMU, 1.5F, 3.0F, &gFilterMadgwickContext);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to warm up Madgwick Filter");
    }

    MotorConfig left_Motor =
    MOTOR_CREATE_CONF (TIM8, TIM_CHANNEL_1, DMA1_Stream0, DMA_REQUEST_TIM8_CH1);
    PWMConfig left_Servo = PWM_CREATE_CONF (TIM13, TIM_CHANNEL_1, 50, TRUE);
    status = ActuatorsInit (left_Servo, left_Motor);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to init Actuators");
    }

    /*
     *
     * Setup FreeRTOS Tasks
     *
     * NOTE: Once a FreeRTOS task is created ALL interrupts will be disabled until the scheduler is started. So functions
     * like HAL_Delay will not work.
     */
    LOG_INFO ("Starting FreeRTOS");
    uint16_t taskPriority = 2;
    BaseType_t taskStatus = xTaskCreate (
    TaskMotionControlUpdate, "Motion Control Update Task",
    configMINIMAL_STACK_SIZE, NULL, taskPriority, &gpTaskMotionControlUpdate);

    if (taskStatus != pdPASS) {
        LOG_ERROR ("Failed to create motion control update task");
    }

    /*
     * NOTE: Give the main control loop a lower priority than the motion
     * control update task (higher number == more important).
     */
    taskPriority = 1;
    taskStatus   = xTaskCreate (
    TaskMainLoop, "Main Control Loop Task", configMINIMAL_STACK_SIZE, NULL,
    taskPriority, NULL);

    if (taskStatus != pdPASS) {
        LOG_ERROR ("Failed to to create main control loop task");
    }

    vTaskStartScheduler ();

    while (1) {
    }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config (void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Supply configuration update enable
     */
    HAL_PWREx_ConfigSupply (PWR_DIRECT_SMPS_SUPPLY);

    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG (PWR_REGULATOR_VOLTAGE_SCALE1);

    while (!__HAL_PWR_GET_FLAG (PWR_FLAG_VOSRDY)) {
    }

    /** Initializes the RCC Oscillators according to the specified
     * parameters in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType =
    RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
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
        Error_Handler ();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType =
    RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
    RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

    if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler ();
    }
    HAL_RCC_MCOConfig (RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init (void) {

    /* SPI2 parameter configuration*/
    hspi2.Instance               = SPI2;
    hspi2.Init.Mode              = SPI_MODE_MASTER;
    hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi2.Init.NSS               = SPI_NSS_HARD_OUTPUT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial     = 0x0;
    hspi2.Init.NSSPMode          = SPI_NSS_PULSE_ENABLE;
    hspi2.Init.NSSPolarity       = SPI_NSS_POLARITY_LOW;
    hspi2.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;
    hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi2.Init.MasterKeepIOState      = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    hspi2.Init.IOSwap                 = SPI_IO_SWAP_DISABLE;
    if (HAL_SPI_Init (&hspi2) != HAL_OK) {
        Error_Handler ();
    }
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init (void) {
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE ();
    __HAL_RCC_GPIOA_CLK_ENABLE ();
    __HAL_RCC_GPIOH_CLK_ENABLE ();
    __HAL_RCC_GPIOF_CLK_ENABLE ();
    __HAL_RCC_GPIOJ_CLK_ENABLE ();

    /*Configure GPIO pin : CEC_CK_MCO1_Pin */
    GPIO_InitStruct.Pin       = CEC_CK_MCO1_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
    HAL_GPIO_Init (CEC_CK_MCO1_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM4 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to
 * increment a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM4) {
        HAL_IncTick ();
    }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler (void) {
    __disable_irq ();
    while (1) {
    }
}
