#include "periphs/gpio.h"
#include "common.h"
#include "conf/conf.h"
#include "hal.h"
#include "log/logger.h"

static GPIO_TypeDef* gPorts[] = { GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF,
                                  GPIOG, GPIOH, GPIOI, GPIOJ, GPIOK };
static uint32_t gPins[]       = { GPIO_PIN_0,  GPIO_PIN_1,  GPIO_PIN_2,
                                  GPIO_PIN_3,  GPIO_PIN_4,  GPIO_PIN_5,
                                  GPIO_PIN_6,  GPIO_PIN_7,  GPIO_PIN_8,
                                  GPIO_PIN_9,  GPIO_PIN_10, GPIO_PIN_11,
                                  GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14,
                                  GPIO_PIN_15 };

static GPIOSPI_t gSPI[eSPI_BUS_ID_MAX]    = { 0 };
static GPIOUART_t gUART[eUART_BUS_ID_MAX] = { 0 };
static GPIOI2C_t gI2C[eI2C_BUS_ID_MAX]    = { 0 };
static GPIOTimer_t gTimers[eTIMER_MAX_ID] = { 0 };

#define GET_PORT(GPIO_ID) gPorts[GPIO_ID2PORTIDX ((GPIO_ID))]
#define GET_PIN(GPIO_ID)  gPins[GPIO_ID2PINIDX ((GPIO_ID))]
#define REG_SPI(BUS_ID, SCK, MISO, MOSI, NSS, ALTERNATE)           \
    do {                                                           \
        gSPI[SPI_BUS_ID2IDX (BUS_ID)].pMISOPort = GET_PORT (MISO); \
        gSPI[SPI_BUS_ID2IDX (BUS_ID)].pMOSIPort = GET_PORT (MOSI); \
        gSPI[SPI_BUS_ID2IDX (BUS_ID)].pSCKPort  = GET_PORT (SCK);  \
        gSPI[SPI_BUS_ID2IDX (BUS_ID)].pNSSPort  = GET_PORT (NSS);  \
        gSPI[SPI_BUS_ID2IDX (BUS_ID)].nssPin    = GET_PIN (NSS);   \
        gSPI[SPI_BUS_ID2IDX (BUS_ID)].misoPin   = GET_PIN (MISO);  \
        gSPI[SPI_BUS_ID2IDX (BUS_ID)].mosiPin   = GET_PIN (MOSI);  \
        gSPI[SPI_BUS_ID2IDX (BUS_ID)].sckPin    = GET_PIN (SCK);   \
        gSPI[SPI_BUS_ID2IDX (BUS_ID)].alternate = (ALTERNATE);     \
    } while (0)
#define REG_UART(BUS_ID, RX, TX, ALTERNATE)                        \
    do {                                                           \
        gUART[UART_BUS_ID2IDX (BUS_ID)].pRXPort   = GET_PORT (RX); \
        gUART[UART_BUS_ID2IDX (BUS_ID)].pTXPort   = GET_PORT (TX); \
        gUART[UART_BUS_ID2IDX (BUS_ID)].rxPin     = GET_PIN (RX);  \
        gUART[UART_BUS_ID2IDX (BUS_ID)].txPin     = GET_PIN (TX);  \
        gUART[UART_BUS_ID2IDX (BUS_ID)].alternate = (ALTERNATE);   \
    } while (0)
#define REG_I2C(BUS_ID, SDA, SCL, ALTERNATE)                      \
    do {                                                          \
        gI2C[I2C_BUS_ID2IDX (BUS_ID)].pSDAPort  = GET_PORT (SDA); \
        gI2C[I2C_BUS_ID2IDX (BUS_ID)].pSCLPort  = GET_PORT (SCL); \
        gI2C[I2C_BUS_ID2IDX (BUS_ID)].sdaPin    = GET_PIN (SDA);  \
        gI2C[I2C_BUS_ID2IDX (BUS_ID)].sclPin    = GET_PIN (SCL);  \
        gI2C[I2C_BUS_ID2IDX (BUS_ID)].alternate = (ALTERNATE);    \
    } while (0)
#define REG_TIMER(TIMER_ID, TIMER_GPIO, ALTERNATE)                          \
    do {                                                                    \
        gTimers[TIMER_ID2IDX (TIMER_ID)].pPort     = GET_PORT (TIMER_GPIO); \
        gTimers[TIMER_ID2IDX (TIMER_ID)].pin       = GET_PIN (TIMER_GPIO);  \
        gTimers[TIMER_ID2IDX (TIMER_ID)].alternate = (ALTERNATE);           \
    } while (0)

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

eSTATUS_t GPIOInitSPI (eBUS_ID_t busId, GPIOSPI_t* pOutSPI) {

    static BOOL_t initialized = FALSE;
    if (initialized == FALSE) {
        REG_SPI (eSPI_1_BUS_ID, eGPIO_A_5_ID, eGPIO_A_6_ID, eGPIO_A_7_ID, eGPIO_C_4_ID, GPIO_AF5_SPI1);
        REG_SPI (eSPI_3_BUS_ID, eGPIO_C_10_ID, eGPIO_C_11_ID, eGPIO_C_12_ID, eGPIO_A_15_ID, GPIO_AF6_SPI3);
        REG_SPI (eSPI_5_BUS_ID, eGPIO_F_7_ID, eGPIO_F_8_ID, eGPIO_F_9_ID, eGPIO_F_10_ID, GPIO_AF5_SPI5);
        initialized = TRUE;
    }

    if (BUS_ID_IS_SPI (busId) == FALSE) {
        return eSTATUS_FAILURE;
    }

    GPIOSPI_t* pSPI = &gSPI[SPI_BUS_ID2IDX (busId)];
    if (pSPI->pSCKPort == NULL || pSPI->pMISOPort == NULL ||
        pSPI->pMOSIPort == NULL || pSPI->pNSSPort == NULL) {
        return eSTATUS_FAILURE;
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

eSTATUS_t GPIOInitOutput (GPIO_TypeDef* pPort, uint16_t pin, GPIOOutput_t* pOutGPIO) {

    if (pPort == NULL) {
        LOG_ERROR ("Invalid GPIO port");
        return eSTATUS_FAILURE;
    }

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Mode             = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull             = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Pin              = pin;

    if (pOutGPIO != NULL) {
        pOutGPIO->pPort = pPort;
        pOutGPIO->pin   = pin;
    }

    HAL_GPIO_Init (pPort, &GPIO_InitStruct);
    return eSTATUS_SUCCESS;
}