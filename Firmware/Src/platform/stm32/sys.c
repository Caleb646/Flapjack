#include <stdint.h>

#include "common.h"

#include "drivers/core/sys.h"

#include "platform/platform.h"

#include "targets/target.h"

FJ_DEFINE_SHARED (Core_t, e_Cores[]) = {
        [CORE_ID_TO_INDEX(CM7_CPUID)] = {
            .id       = CM7_CPUID,
            .pendIrqId = PendSV_IRQn,
            .sevIrqId  = CM4_SEV_IRQn,
        },
        [CORE_ID_TO_INDEX(CM4_CPUID)] = {
            .id       = CM4_CPUID,
            .pendIrqId = PendSV_IRQn,
            .sevIrqId  = CM7_SEV_IRQn,
        },
    };
FJ_DEFINE_SHARED (uint8_t, e_nCores)  = sizeof (e_Cores) / sizeof (Core_t);
FJ_DEFINE_SHARED (System_t, e_System) = { 0 };

void SysTick_Handler (void) {

    HAL_IncTick ();
}

/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler (void) {

    while (1) {
    }
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler (void) {

    while (1) {
    }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler (void) {

    while (1) {
    }
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler (void) {

    while (1) {
    }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler (void) {

    while (1) {
    }
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler (void) {
}

static eSTATUS_t Stm32_System_InitClock (System_t* pSystem) {

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
        return eSTATUS_FAIL;
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
        return eSTATUS_FAIL;
    }
    HAL_RCC_MCOConfig (RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
    return eSTATUS_OK;
}

static eSTATUS_t Stm32_System_InitCore (System_t* pSystem, uint32_t coreId) {

    eSTATUS_t status = eSTATUS_OK;
    if (HAL_Init () != HAL_OK) {
        status |= eSTATUS_FAIL;
    }

    Core_t* pCore = &e_Cores[CORE_ID_TO_INDEX (coreId)];

    HAL_NVIC_SetPriority (pCore->pendIrqId, 15, 0);
    HAL_NVIC_EnableIRQ (pCore->pendIrqId);

    HAL_NVIC_SetPriority (pCore->sevIrqId, 9, 9);
    HAL_NVIC_EnableIRQ (pCore->sevIrqId);

    return status;
}

static eSTATUS_t Stm32_System_InitPeriphClks (System_t* pSystem) {

    RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit = { 0 };
    // select clock source for uart peripherals
    RCC_PeriphClkInit.PeriphClockSelection      = RCC_PERIPHCLK_USART16 | RCC_PERIPHCLK_USART234578;
    RCC_PeriphClkInit.Usart16ClockSelection     = RCC_USART16CLKSOURCE_D2PCLK2;
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

    return eSTATUS_OK;
}

eSTATUS_t Plat_System_Init (System_t* pSystem, uint32_t coreId, bool isPrimary) {

    __HAL_RCC_SYSCFG_CLK_ENABLE ();
    __HAL_RCC_HSEM_CLK_ENABLE ();
    eSTATUS_t status = Stm32_System_InitCore (pSystem, coreId);
    if (isPrimary) {
        status |= Stm32_System_InitClock (pSystem);
        status |= Stm32_System_InitPeriphClks (pSystem);
    }

    // Enable core debug access and trace unit
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // Enable DWT cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;

    return status;
}

uint32_t Plat_CurrentCore_GetId (void) {
    return HAL_GetCurrentCPUID ();
}