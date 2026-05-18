#include "test_dshot.h"

#include "hal.h"
#include "target.h"

#include "core/log/logger.h"

#include "unity.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef s_huart = { 0 };

void Uart_PutChar (void* p, char c) {
    HAL_UART_Transmit (&s_huart, (uint8_t const*)&c, sizeof (c), 100);
}

void SysTick_Handler (void) {
    HAL_IncTick ();
}

static void HwTest_UartInit (void) {

    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct  = { 0 };
    PeriphClkInitStruct.PeriphClockSelection      = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart16ClockSelection     = RCC_USART16CLKSOURCE_D2PCLK2;
    PeriphClkInitStruct.Usart234578ClockSelection = 0U;
    HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct);
    __HAL_RCC_USART1_CLK_ENABLE ();

    __HAL_RCC_USART1_CLK_ENABLE ();
    __HAL_RCC_GPIOA_CLK_ENABLE ();

    GPIO_InitTypeDef gpio = { 0 };
    gpio.Pin              = GPIO_PIN_9 | GPIO_PIN_10;
    gpio.Mode             = GPIO_MODE_AF_PP;
    gpio.Pull             = GPIO_NOPULL;
    gpio.Speed            = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate        = GPIO_AF7_USART1;
    HAL_GPIO_Init (GPIOA, &gpio);

    s_huart.Instance          = USART1;
    s_huart.Init.BaudRate     = 230400;
    s_huart.Init.WordLength             = UART_WORDLENGTH_8B;
    s_huart.Init.StopBits               = UART_STOPBITS_1;
    s_huart.Init.Parity                 = UART_PARITY_NONE;
    s_huart.Init.Mode                   = UART_MODE_TX_RX;
    s_huart.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    s_huart.Init.OverSampling           = UART_OVERSAMPLING_16;
    s_huart.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    s_huart.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    s_huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init (&s_huart);


    HAL_UARTEx_SetTxFifoThreshold (&s_huart, UART_TXFIFO_THRESHOLD_1_8);
    HAL_UARTEx_SetRxFifoThreshold (&s_huart, UART_RXFIFO_THRESHOLD_1_8);
    HAL_UARTEx_DisableFifoMode (&s_huart);
}

static void SystemClock_Config (void);

extern void __SHARED_MEM_BSS_START__;
extern void __SHARED_MEM_BSS_END__;
extern void __SHARED_MEM_DATA_START__;
extern void __SHARED_MEM_DATA_END__;
extern void __SHARED_MEM_DATA_FLASH_START__;

int main (void) {

#ifdef CORE_CM7

    __HAL_RCC_SYSCFG_CLK_ENABLE ();
    __HAL_RCC_HSEM_CLK_ENABLE ();

    HAL_Init ();
    HAL_NVIC_EnableIRQ (SysTick_IRQn);
    SystemClock_Config ();

    uint32_t const bssStart = (uint32_t)&__SHARED_MEM_BSS_START__;
    uint32_t const bssEnd   = (uint32_t)&__SHARED_MEM_BSS_END__;
    memset ((void*)bssStart, 0, bssEnd - bssStart);

    uint32_t const dataStart      = (uint32_t)&__SHARED_MEM_DATA_START__;
    uint32_t const dataEnd        = (uint32_t)&__SHARED_MEM_DATA_END__;
    uint32_t const dataFlashStart = (uint32_t)&__SHARED_MEM_DATA_FLASH_START__;
    memcpy ((void*)dataStart, (void*)dataFlashStart, dataEnd - dataStart);

    HwTest_UartInit ();
    init_printf (NULL, Uart_PutChar);
    HAL_Delay (500);
    LOG_INFO ("Starting HIL Tests");
    HAL_Delay (500);

    UNITY_BEGIN ();
    LOG_INFO ("Running DShot Init Test");
    RUN_TEST (test_hil_dshot_init);
    LOG_INFO ("Running DShot period test");
    RUN_TEST (test_hil_dshot_bit_period);
    // // RUN_TEST (test_hil_dshot_bit0_pulse_width);
    // // RUN_TEST (test_hil_dshot_bit1_pulse_width);
    // // RUN_TEST (test_hil_dshot_frame_length);
    UNITY_END ();

#endif

    while (1) {
        __WFI ();
    }
}

static void SystemClock_Config (void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

#if defined(USE_PWR_DIRECT_SMPS_SUPPLY)
    HAL_PWREx_ConfigSupply (PWR_DIRECT_SMPS_SUPPLY);
#endif
#if defined(USE_PWR_LDO_SUPPLY)
    HAL_PWREx_ConfigSupply (PWR_LDO_SUPPLY);
#endif

    __HAL_PWR_VOLTAGESCALING_CONFIG (PWR_REGULATOR_VOLTAGE_SCALE1);
    while (!__HAL_PWR_GET_FLAG (PWR_FLAG_VOSRDY)) {
    }

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
    if (HSE_VALUE == 0U) {
        RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
        RCC_OscInitStruct.OscillatorType &= ~RCC_OSCILLATORTYPE_HSE;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    }
    if (HAL_RCC_OscConfig (&RCC_OscInitStruct) != HAL_OK) {
        while (1) {
        }
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
    if (HSE_VALUE == 0U) {
        RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
        RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    }
    if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        while (1) {
        }
    }
}
