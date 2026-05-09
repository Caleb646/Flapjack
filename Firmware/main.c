#include <stdio.h>
#include <string.h>

#include "flight.h"
#include "hal.h"
#include "scheduler.h"
#include "target.h"

#include "core/core.h"

#include "fc/rc.h"

#include "mc/filter.h"
#include "mc/pid.h"

#include "drivers/dma.h"
#include "drivers/io/gpio.h"

#include "drivers/rx/rx.h"

#include "drivers/sensors/mag/mag.h"

#include "device/imu/imu.h"
#include "device/serial/serial.h"

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U)
#endif

void SysTick_Handler (void) {

    HAL_IncTick ();
}

FJ_DEFINE_SHARED (bool volatile, s_IsCM4Stuck)     = false;
FJ_DEFINE_SHARED (bool volatile, s_IsSystemInited) = false;
FJ_DEFINE_SHARED (bool volatile, s_IsCM4Ready)     = false;


#ifdef CORE_CM7

void SystemClock_Config (void);

extern void __SHARED_MEM_BSS_START__;
extern void __SHARED_MEM_BSS_END__;
extern void __SHARED_MEM_DATA_START__;
extern void __SHARED_MEM_DATA_END__;
extern void __SHARED_MEM_DATA_FLASH_START__;

int main (void) {

    HAL_Init ();
    HAL_NVIC_EnableIRQ (SysTick_IRQn);
    __HAL_RCC_HSEM_CLK_ENABLE ();
    SystemClock_Config ();

    uint32_t const bssStart = (uint32_t)&__SHARED_MEM_BSS_START__;
    uint32_t const bssEnd   = (uint32_t)&__SHARED_MEM_BSS_END__;
    memset ((void*)bssStart, 0, bssEnd - bssStart);

    uint32_t const dataStart      = (uint32_t)&__SHARED_MEM_DATA_START__;
    uint32_t const dataEnd        = (uint32_t)&__SHARED_MEM_DATA_END__;
    uint32_t const dataFlashStart = (uint32_t)&__SHARED_MEM_DATA_FLASH_START__;
    memcpy ((void*)dataStart, (void*)dataFlashStart, dataEnd - dataStart);

    /* Take HSEM */
    HAL_HSEM_FastTake (HSEM_ID_0);
    /* Release HSEM in order to notify the CPU2(CM4) */
    HAL_HSEM_Release (HSEM_ID_0, 0);
    /* Wait until CPU2 wakes up from stop mode */
    while (__HAL_RCC_GET_FLAG (RCC_FLAG_D2CKRDY) == RESET) {
    }
    __HAL_RCC_SYSCFG_CLK_ENABLE ();

    if (STATUS_FAIL (Spi_InitSystem ())) {
        CriticalErrorHandler ();
    }
    if (STATUS_FAIL (Uart_InitSystem ())) {
        CriticalErrorHandler ();
    }
    if (STATUS_FAIL (Core_Init ())) {
        CriticalErrorHandler ();
    }
    if (STATUS_FAIL (SerialDebug_Init ())) {
        CriticalErrorHandler ();
    }
    s_IsSystemInited = true;
    while (!s_IsCM4Ready) {
        // allow cm4 to initialize logger
    };

    if (STATUS_FAIL (IMU_Init ())) {
        LOG_ERROR ("Failed to init IMU");
        CriticalErrorHandler ();
    }

    if (STATUS_FAIL (Mag_Init ())) {
        LOG_ERROR ("Failed to init MAG");
        CriticalErrorHandler ();
    }

    if (STATUS_FAIL (Pid_Init ())) {
        LOG_ERROR ("Failed to init PID");
        CriticalErrorHandler ();
    }

    if (STATUS_FAIL (Fc_Init ())) {
        LOG_ERROR ("Failed to init filter");
        CriticalErrorHandler ();
    }

    if (STATUS_FAIL (Rc_Init ())) {
        LOG_ERROR ("Failed to init RC");
        CriticalErrorHandler ();
    }

    LOG_INFO ("Starting scheduler");
    Scheduler_Main (CM7_IDX, CFG_LOOP_UPDATE_RATE_HZ);
}

void SystemClock_Config (void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

#if defined(USE_PWR_DIRECT_SMPS_SUPPLY)
#warning "Using SMPS power supply. Ensure that the board is configured for SMPS and not LDO!"
    HAL_PWREx_ConfigSupply (PWR_DIRECT_SMPS_SUPPLY);
#endif /* USE_PWR_DIRECT_SMPS_SUPPLY && SMPS */

#if defined(USE_PWR_LDO_SUPPLY)
#warning "Using LDO power supply. Ensure that the board is configured for LDO and not SMPS!"
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
#if HSE_VALUE == 0U
#warning "HSE_VALUE is not defined or set to 0. HSE oscillator will be disabled. Ensure that HSE_VALUE is set correctly if using an external crystal."
#endif
    if (HSE_VALUE == 0U) {
        RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
        RCC_OscInitStruct.OscillatorType &= ~RCC_OSCILLATORTYPE_HSE;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    }
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

    // If HSE is not used HSI (64MHz) is used as the HCLK. APB1 and APB2 max is 50MHz, so set dividers to 2 (32MHz)
    if (HSE_VALUE == 0U) {
        RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
        RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    }


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

    __HAL_RCC_SYSCFG_CLK_ENABLE ();
    // enable CM4_SEV_IRQn so CM7 can send SEV to CM4
    // HAL_NVIC_SetPriority (CM7_SEV_IRQn, 9, 9);
    // HAL_NVIC_EnableIRQ (CM7_SEV_IRQn);

    while (!s_IsSystemInited) {
        // wait until CM7 has initialized the system before proceeding with any
        // initialization that may rely on the system being initialized
    }

    if (Core_Init () != eSTATUS_SUCCESS) {
        s_IsCM4Stuck = true;
        CriticalErrorHandler ();
    }
    s_IsCM4Ready = true;

    // TODO: remove temporary
    // uint32_t tempStart = GetMilliseconds ();
    // while (true) {
    //     SyncProcessTasks ();
    //     if (GetMilliseconds () - tempStart >= 1000) {
    //         tempStart = GetMilliseconds ();
    //         LOG_INFO ("System initialized. Starting main loop.");
    //     }
    // }

    if (STATUS_FAIL (Rx_Init ())) {
        LOG_ERROR ("Failed to initialize RX");
        CriticalErrorHandler ();
    }

    LOG_INFO ("Starting scheduler");
    Scheduler_Main (CM4_IDX, CFG_LOOP_UPDATE_RATE_HZ);
}

#endif // CORE_CM4