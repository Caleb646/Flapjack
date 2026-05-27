#include "test_dshot.h"

#include "hal.h"
#include "target.h"
#include "platform.h"

#include "core/core.h"

#include "unity.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef s_huart = { 0 };

void Uart_PutChar (void* p, char c) {
    HAL_UART_Transmit (&s_huart, (uint8_t const*)&c, sizeof (c), 100);
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

    s_huart.Instance                    = USART1;
    s_huart.Init.BaudRate               = 230400;
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

int main (void) {

    if(Platform_Init() != 0) {
        CriticalErrorHandler ();
    }

#ifdef CORE_CM7

    HwTest_UartInit ();
    init_printf (NULL, Uart_PutChar);

    Delay (500);
    LOG_INFO ("Starting HIL Tests");
    Delay (500);

    UNITY_BEGIN ();
    LOG_INFO ("Running DShot Init Test");
    RUN_TEST (test_hil_dshot_init);
    LOG_INFO ("Running DShot period test");
    RUN_TEST (test_hil_dshot_bit_period);
    // // RUN_TEST (test_hil_dshot_bit0_pulse_width);
    // // RUN_TEST (test_hil_dshot_bit1_pulse_width);
    // // RUN_TEST (test_hil_dshot_frame_length);
    UNITY_END ();

    Delay (50000);

#endif

    while (1) {
        __WFI ();
    }
}