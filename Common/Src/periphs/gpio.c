#include "periphs/gpio.h"
#include "common.h"
#include "hal.h"
#include "log/logger.h"


eSTATUS_t GPIOInitUART (USART_TypeDef* pInstance) {

    if (pInstance == NULL) {
        LOG_ERROR ("Invalid UART instance");
        return eSTATUS_FAILURE;
    }

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Mode             = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_HIGH;

    if (pInstance == USART1) {
        UART1_GPIO_CLK_ENABLE ();
        GPIO_InitStruct.Pin       = UART1_RX_GPIO_Pin | UART1_TX_GPIO_Pin;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init (UART1_GPIO_Port, &GPIO_InitStruct);

    } else if (pInstance == USART2) {
        UART2_GPIO_CLK_ENABLE ();
        GPIO_InitStruct.Pin       = UART2_RX_GPIO_Pin | UART2_TX_GPIO_Pin;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init (UART2_GPIO_Port, &GPIO_InitStruct);

    } else if (pInstance == USART3) {
        UART3_GPIO_CLK_ENABLE ();
        GPIO_InitStruct.Pin       = UART3_RX_GPIO_Pin | UART3_TX_GPIO_Pin;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init (UART3_GPIO_Port, &GPIO_InitStruct);

    } else {
        LOG_ERROR ("Unsupported UART instance");
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t GPIOInitSPI (SPI_TypeDef* pInstance) {

    if (pInstance == NULL) {
        LOG_ERROR ("Invalid SPI instance");
        return eSTATUS_FAILURE;
    }

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Mode             = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_HIGH;

    if (pInstance == SPI1) {
        SPI1_DATA_GPIO_CLK_ENABLE ();
        SPI1_NSS_GPIO_CLK_ENABLE ();
        // SPI1 Data Pins
        GPIO_InitStruct.Pin       = SPI1_DATA_Pins;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init (SPI1_DATA_GPIO_Port, &GPIO_InitStruct);
        // SPI1 NSS Pin
        GPIO_InitStruct.Pin       = SPI1_NSS_GPIO_Pin;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init (SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

    } else if (pInstance == SPI3) {
        SPI3_DATA_GPIO_CLK_ENABLE ();
        SPI3_NSS_GPIO_CLK_ENABLE ();
        // SPI3 Data Pins
        GPIO_InitStruct.Pin       = SPI3_DATA_Pins;
        GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
        HAL_GPIO_Init (SPI3_DATA_GPIO_Port, &GPIO_InitStruct);
        // SPI3 NSS Pin
        GPIO_InitStruct.Pin       = SPI3_NSS_GPIO_Pin;
        GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
        HAL_GPIO_Init (SPI3_NSS_GPIO_Port, &GPIO_InitStruct);

    } else if (pInstance == SPI5) {
        SPI5_DATA_GPIO_CLK_ENABLE ();
        SPI5_NSS_GPIO_CLK_ENABLE ();
        // SPI5 Data Pins
        GPIO_InitStruct.Pin       = SPI5_DATA_Pins;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
        HAL_GPIO_Init (SPI5_DATA_GPIO_Port, &GPIO_InitStruct);
        // SPI5 NSS Pin
        GPIO_InitStruct.Pin       = SPI5_NSS_GPIO_Pin;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
        HAL_GPIO_Init (SPI5_NSS_GPIO_Port, &GPIO_InitStruct);

    } else {
        LOG_ERROR ("Unsupported SPI instance");
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t GPIOInitEXTI (IRQn_Type irq, uint32_t pin) {

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Mode             = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_LOW;

    if (irq == EXTI3_IRQn) {
        EXTI3_GPIO_CLK_ENABLE ();
        GPIO_InitStruct.Pin = EXTI3_GPIO_Pin;
        HAL_GPIO_Init (EXTI3_GPIO_Port, &GPIO_InitStruct);

    } else if (irq == EXTI9_5_IRQn && pin == EXTI5_GPIO_Pin) {
        EXTI5_GPIO_CLK_ENABLE ();
        GPIO_InitStruct.Pin = EXTI5_GPIO_Pin;
        HAL_GPIO_Init (EXTI5_GPIO_Port, &GPIO_InitStruct);

    } else if (irq == EXTI9_5_IRQn && pin == EXTI6_GPIO_Pin) {
        EXTI6_GPIO_CLK_ENABLE ();
        GPIO_InitStruct.Pin = EXTI6_GPIO_Pin;
        HAL_GPIO_Init (EXTI6_GPIO_Port, &GPIO_InitStruct);

    } else {
        LOG_ERROR ("Unsupported EXTI IRQ");
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t GPIOInitTimer (TIM_TypeDef* pInstance, uint32_t channelId) {
    if (pInstance == NULL) {
        LOG_ERROR ("Invalid Timer instance");
        return eSTATUS_FAILURE;
    }

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Mode             = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_HIGH;

    if (pInstance == TIM5) {
        TIM5_GPIO_CLK_ENABLE ();
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;

        if (channelId == TIM_CHANNEL_1) {
            GPIO_InitStruct.Pin = TIM5_CH1_GPIO_Pin;
            HAL_GPIO_Init (TIM5_CH1_GPIO_Port, &GPIO_InitStruct);

        } else if (channelId == TIM_CHANNEL_2) {
            GPIO_InitStruct.Pin = TIM5_CH2_GPIO_Pin;
            HAL_GPIO_Init (TIM5_CH2_GPIO_Port, &GPIO_InitStruct);

        } else if (channelId == TIM_CHANNEL_3) {
            GPIO_InitStruct.Pin = TIM5_CH3_GPIO_Pin;
            HAL_GPIO_Init (TIM5_CH3_GPIO_Port, &GPIO_InitStruct);

        } else if (channelId == TIM_CHANNEL_4) {
            GPIO_InitStruct.Pin = TIM5_CH4_GPIO_Pin;
            HAL_GPIO_Init (TIM5_CH4_GPIO_Port, &GPIO_InitStruct);
        } else {
            LOG_ERROR ("Unsupported Timer channel");
            return eSTATUS_FAILURE;
        }

    } else if (pInstance == TIM8) {
        TIM8_GPIO_CLK_ENABLE ();
        if (channelId == TIM_CHANNEL_1) {
            GPIO_InitStruct.Pin = TIM8_CH1_GPIO_Pin;
            HAL_GPIO_Init (TIM8_CH1_GPIO_Port, &GPIO_InitStruct);

        } else if (channelId == TIM_CHANNEL_2) {
            GPIO_InitStruct.Pin = TIM8_CH2_GPIO_Pin;
            HAL_GPIO_Init (TIM8_CH2_GPIO_Port, &GPIO_InitStruct);

        } else if (channelId == TIM_CHANNEL_3) {
            GPIO_InitStruct.Pin = TIM8_CH3_GPIO_Pin;
            HAL_GPIO_Init (TIM8_CH3_GPIO_Port, &GPIO_InitStruct);

        } else if (channelId == TIM_CHANNEL_4) {
            GPIO_InitStruct.Pin = TIM8_CH4_GPIO_Pin;
            HAL_GPIO_Init (TIM8_CH4_GPIO_Port, &GPIO_InitStruct);

        } else {
            LOG_ERROR ("Unsupported Timer channel");
            return eSTATUS_FAILURE;
        }
    } else if (pInstance == TIM12) {
        TIM12_GPIO_CLK_ENABLE ();
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM12;

        if (channelId == TIM_CHANNEL_1) {
            GPIO_InitStruct.Pin = TIM12_CH1_GPIO_Pin;
            HAL_GPIO_Init (TIM12_CH1_GPIO_Port, &GPIO_InitStruct);

        } else if (channelId == TIM_CHANNEL_2) {
            GPIO_InitStruct.Pin = TIM12_CH2_GPIO_Pin;
            HAL_GPIO_Init (TIM12_CH2_GPIO_Port, &GPIO_InitStruct);

        } else {
            LOG_ERROR ("Unsupported Timer channel");
            return eSTATUS_FAILURE;
        }

    } else {
        LOG_ERROR ("Unsupported Timer instance");
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t GPIOInitI2C (I2C_TypeDef* pInstance) {

    if (pInstance == NULL) {
        LOG_ERROR ("Invalid I2C instance");
        return eSTATUS_FAILURE;
    }

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Mode             = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_HIGH;

    if (pInstance == I2C1) {
        I2C1_GPIO_CLK_ENABLE ();
        GPIO_InitStruct.Pin       = I2C1_SCL_GPIO_Pin | I2C1_SDA_GPIO_Pin;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init (I2C1_GPIO_Port, &GPIO_InitStruct);

    } else {
        LOG_ERROR ("Unsupported I2C instance");
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}