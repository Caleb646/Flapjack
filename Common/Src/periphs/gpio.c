#include "periphs/gpio.h"
#include "common.h"
#include "hal.h"
#include "log/logger.h"


eSTATUS_t GPIOInitUART (USART_TypeDef* pInstance, GPIOUART_t* pOutUART) {

    if (pInstance == NULL) {
        LOG_ERROR ("Invalid UART instance");
        return eSTATUS_FAILURE;
    }

    GPIO_TypeDef* pTXPort            = NULL;
    GPIO_TypeDef* pRXPort            = NULL;
    uint16_t txPin                   = 0U;
    uint16_t rxPin                   = 0U;
    uint32_t alternate               = 0U;
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Mode             = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_HIGH;

    if (pInstance == USART1) {
        UART1_GPIO_CLK_ENABLE ();
        pTXPort   = UART1_GPIO_Port;
        pRXPort   = UART1_GPIO_Port;
        txPin     = UART1_TX_GPIO_Pin;
        rxPin     = UART1_RX_GPIO_Pin;
        alternate = GPIO_AF7_USART1;

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

    if (pOutUART != NULL) {
        pOutUART->pTXPort = pTXPort;
        pOutUART->pRXPort = pRXPort;
        pOutUART->txPin   = txPin;
        pOutUART->rxPin   = rxPin;
    }

    GPIO_InitStruct.Pin       = txPin;
    GPIO_InitStruct.Alternate = alternate;
    HAL_GPIO_Init (pTXPort, &GPIO_InitStruct);
    GPIO_InitStruct.Pin       = rxPin;
    GPIO_InitStruct.Alternate = alternate;
    HAL_GPIO_Init (pRXPort, &GPIO_InitStruct);
    return eSTATUS_SUCCESS;
}

eSTATUS_t GPIOInitSPI (SPI_TypeDef* pInstance, GPIOSPI_t* pOutSPI) {

    if (pInstance == NULL) {
        LOG_ERROR ("Invalid SPI instance");
        return eSTATUS_FAILURE;
    }

    GPIO_TypeDef* pDataPort = NULL;
    GPIO_TypeDef* pNSSPort  = NULL;
    uint16_t nssPin         = 0U;
    uint16_t dataPins       = 0U;
    uint32_t alternate      = 0U;

    if (pInstance == SPI1) {
        SPI1_DATA_GPIO_CLK_ENABLE ();
        SPI1_NSS_GPIO_CLK_ENABLE ();
        pDataPort = SPI1_DATA_GPIO_Port;
        pNSSPort  = SPI1_NSS_GPIO_Port;
        nssPin    = SPI1_NSS_GPIO_Pin;
        dataPins  = SPI1_DATA_Pins;
        alternate = GPIO_AF5_SPI1;

    } else if (pInstance == SPI3) {
        SPI3_DATA_GPIO_CLK_ENABLE ();
        SPI3_NSS_GPIO_CLK_ENABLE ();
        pDataPort = SPI3_DATA_GPIO_Port;
        pNSSPort  = SPI3_NSS_GPIO_Port;
        nssPin    = SPI3_NSS_GPIO_Pin;
        dataPins  = SPI3_DATA_Pins;
        alternate = GPIO_AF6_SPI3;

    } else if (pInstance == SPI5) {
        SPI5_DATA_GPIO_CLK_ENABLE ();
        SPI5_NSS_GPIO_CLK_ENABLE ();
        pDataPort = SPI5_DATA_GPIO_Port;
        pNSSPort  = SPI5_NSS_GPIO_Port;
        nssPin    = SPI5_NSS_GPIO_Pin;
        dataPins  = SPI5_DATA_Pins;
        alternate = GPIO_AF5_SPI5;

    } else {
        LOG_ERROR ("Unsupported SPI instance");
        return eSTATUS_FAILURE;
    }

    if (pOutSPI != NULL) {
        pOutSPI->pNSSPort = pNSSPort;
        pOutSPI->nssPin   = nssPin;
    }

    GPIO_InitTypeDef GPIO_InitStruct = { 0U };
    GPIO_InitStruct.Mode             = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin              = dataPins;
    GPIO_InitStruct.Alternate        = alternate;
    HAL_GPIO_Init (pDataPort, &GPIO_InitStruct);

    GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Pin       = nssPin;
    GPIO_InitStruct.Alternate = alternate;
    HAL_GPIO_Init (pNSSPort, &GPIO_InitStruct);
    return eSTATUS_SUCCESS;
}

eSTATUS_t GPIOInitEXTI (IRQn_Type irq, uint32_t pin, GPIOEXTI_t* pOutExti) {

    GPIO_TypeDef* pPort              = NULL;
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Mode             = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_LOW;

    if (irq == EXTI3_IRQn) {
        EXTI3_GPIO_CLK_ENABLE ();
        GPIO_InitStruct.Pin = EXTI3_GPIO_Pin;
        pPort               = EXTI3_GPIO_Port;

    } else if (irq == EXTI9_5_IRQn && pin == EXTI5_GPIO_Pin) {
        EXTI5_GPIO_CLK_ENABLE ();
        GPIO_InitStruct.Pin = EXTI5_GPIO_Pin;
        pPort               = EXTI5_GPIO_Port;

    } else if (irq == EXTI9_5_IRQn && pin == EXTI6_GPIO_Pin) {
        EXTI6_GPIO_CLK_ENABLE ();
        GPIO_InitStruct.Pin = EXTI6_GPIO_Pin;
        pPort               = EXTI6_GPIO_Port;

    } else {
        LOG_ERROR ("Unsupported EXTI IRQ");
        return eSTATUS_FAILURE;
    }

    if (pOutExti != NULL) {
        pOutExti->pPort = pPort;
        pOutExti->pin   = GPIO_InitStruct.Pin;
    }

    HAL_GPIO_Init (pPort, &GPIO_InitStruct);
    return eSTATUS_SUCCESS;
}

eSTATUS_t GPIOInitTimer (TIM_TypeDef* pInstance, uint32_t channelId, GPIOTimer_t* pOutTimer) {

    if (pInstance == NULL) {
        LOG_ERROR ("Invalid Timer instance");
        return eSTATUS_FAILURE;
    }

    GPIO_TypeDef* pPort              = NULL;
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Mode             = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_HIGH;

    if (pInstance == TIM5) {
        TIM5_GPIO_CLK_ENABLE ();
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;

        if (channelId == TIM_CHANNEL_1) {
            GPIO_InitStruct.Pin = TIM5_CH1_GPIO_Pin;
            pPort               = TIM5_CH1_GPIO_Port;

        } else if (channelId == TIM_CHANNEL_2) {
            GPIO_InitStruct.Pin = TIM5_CH2_GPIO_Pin;
            pPort               = TIM5_CH2_GPIO_Port;

        } else if (channelId == TIM_CHANNEL_3) {
            GPIO_InitStruct.Pin = TIM5_CH3_GPIO_Pin;
            pPort               = TIM5_CH3_GPIO_Port;

        } else if (channelId == TIM_CHANNEL_4) {
            GPIO_InitStruct.Pin = TIM5_CH4_GPIO_Pin;
            pPort               = TIM5_CH4_GPIO_Port;
        }

    } else if (pInstance == TIM8) {
        TIM8_GPIO_CLK_ENABLE ();

        if (channelId == TIM_CHANNEL_1) {
            GPIO_InitStruct.Pin = TIM8_CH1_GPIO_Pin;
            pPort               = TIM8_CH1_GPIO_Port;

        } else if (channelId == TIM_CHANNEL_2) {
            GPIO_InitStruct.Pin = TIM8_CH2_GPIO_Pin;
            pPort               = TIM8_CH2_GPIO_Port;

        } else if (channelId == TIM_CHANNEL_3) {
            GPIO_InitStruct.Pin = TIM8_CH3_GPIO_Pin;
            pPort               = TIM8_CH3_GPIO_Port;

        } else if (channelId == TIM_CHANNEL_4) {
            GPIO_InitStruct.Pin = TIM8_CH4_GPIO_Pin;
            pPort               = TIM8_CH4_GPIO_Port;
        }

    } else if (pInstance == TIM12) {
        TIM12_GPIO_CLK_ENABLE ();
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM12;

        if (channelId == TIM_CHANNEL_1) {
            GPIO_InitStruct.Pin = TIM12_CH1_GPIO_Pin;
            pPort               = TIM12_CH1_GPIO_Port;

        } else if (channelId == TIM_CHANNEL_2) {
            GPIO_InitStruct.Pin = TIM12_CH2_GPIO_Pin;
            pPort               = TIM12_CH2_GPIO_Port;
        }

    } else {
        LOG_ERROR ("Unsupported Timer instance");
        return eSTATUS_FAILURE;
    }

    if (pPort == NULL) {
        LOG_ERROR ("Unsupported Timer channel");
        return eSTATUS_FAILURE;
    }

    if (pOutTimer != NULL) {
        pOutTimer->pPort = pPort;
        pOutTimer->pin   = GPIO_InitStruct.Pin;
    }

    HAL_GPIO_Init (pPort, &GPIO_InitStruct);
    return eSTATUS_SUCCESS;
}

eSTATUS_t GPIOInitI2C (I2C_TypeDef* pInstance, GPIOI2C_t* pOutI2C) {

    if (pInstance == NULL) {
        LOG_ERROR ("Invalid I2C instance");
        return eSTATUS_FAILURE;
    }

    GPIO_TypeDef* pSCLPort           = NULL;
    GPIO_TypeDef* pSDAPort           = NULL;
    uint16_t sclPin                  = 0;
    uint16_t sdaPin                  = 0;
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Mode             = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_HIGH;

    if (pInstance == I2C1) {
        I2C1_GPIO_CLK_ENABLE ();
        GPIO_InitStruct.Pin       = I2C1_SCL_GPIO_Pin | I2C1_SDA_GPIO_Pin;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        pSCLPort                  = I2C1_GPIO_Port;
        pSDAPort                  = I2C1_GPIO_Port;
        sclPin                    = I2C1_SCL_GPIO_Pin;
        sdaPin                    = I2C1_SDA_GPIO_Pin;

    } else {
        LOG_ERROR ("Unsupported I2C instance");
        return eSTATUS_FAILURE;
    }

    if (pOutI2C != NULL) {
        pOutI2C->pSCLPort = pSCLPort;
        pOutI2C->pSDAPort = pSDAPort;
        pOutI2C->sclPin   = sclPin;
        pOutI2C->sdaPin   = sdaPin;
    }

    HAL_GPIO_Init (pSCLPort, &GPIO_InitStruct);
    HAL_GPIO_Init (pSDAPort, &GPIO_InitStruct);
    return eSTATUS_SUCCESS;
}