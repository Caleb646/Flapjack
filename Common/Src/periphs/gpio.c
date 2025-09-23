#include "periphs/gpio.h"
#include "common.h"
#include "conf/conf.h"
#include "hal.h"
#include "log/logger.h"

#define GPIO_NPORTS eGPIO_PORTID_MAX
#define GPIO_NPINS  eGPIO_PINID_MAX
#define PORT_OFFSET 0x400 // each port is offset by 0x400

static IO_t gIOs[GPIO_NPORTS * GPIO_NPINS] = { 0 };

static void GPIOEnablePortClock (eGPIO_ID_t gpioId) {

    IO_t* pIO = GPIOGetIOfromId (gpioId);
    if (pIO == NULL || pIO->pPort == NULL) {
        return;
    }

    GPIO_TypeDef* pPort = pIO->pPort;
    if (pPort == GPIOA) {
        __HAL_RCC_GPIOA_CLK_ENABLE ();
    } else if (pPort == GPIOB) {
        __HAL_RCC_GPIOB_CLK_ENABLE ();
    } else if (pPort == GPIOC) {
        __HAL_RCC_GPIOC_CLK_ENABLE ();
    } else if (pPort == GPIOD) {
        __HAL_RCC_GPIOD_CLK_ENABLE ();
    } else if (pPort == GPIOE) {
        __HAL_RCC_GPIOE_CLK_ENABLE ();
    } else if (pPort == GPIOF) {
        __HAL_RCC_GPIOF_CLK_ENABLE ();
    } else if (pPort == GPIOG) {
        __HAL_RCC_GPIOG_CLK_ENABLE ();
    } else if (pPort == GPIOH) {
        __HAL_RCC_GPIOH_CLK_ENABLE ();
    } else if (pPort == GPIOI) {
        __HAL_RCC_GPIOI_CLK_ENABLE ();
    } else if (pPort == GPIOJ) {
        __HAL_RCC_GPIOJ_CLK_ENABLE ();
    } else if (pPort == GPIOK) {
        __HAL_RCC_GPIOK_CLK_ENABLE ();
    }
}

static void GPIOBaseInit (void) {

    static BOOL_t initialized = FALSE;
    if (initialized == TRUE) {
        return;
    }
    for (uint32_t portId = 0; portId < GPIO_NPORTS; ++portId) {
        for (uint32_t pinId = 0; pinId < GPIO_NPINS; ++pinId) {
            IO_t* pIO    = &gIOs[(portId * GPIO_NPINS) + pinId];
            pIO->pPort   = GPIOA + (portId * PORT_OFFSET);
            pIO->pin     = GPIO_PIN_0 << pinId;
            pIO->ownerId = eDEVICE_ID_NULL;
        }
    }
    initialized = TRUE;
}

IO_t* GPIOGetIOfromId (eGPIO_ID_t gpioId) {

    if (GPIO_ID_IS_GPIO (gpioId) == FALSE) {
        return NULL;
    }
    eGPIO_PORTID_t portIdx = GPIO_ID2PORTIDX (gpioId);
    eGPIO_PINID_t pinIdx   = GPIO_ID2PINIDX (gpioId);
    uint32_t idx           = (portIdx * GPIO_NPINS) + pinIdx;
    if (idx >= (GPIO_NPORTS * GPIO_NPINS)) {
        return NULL;
    }
    return &gIOs[(portIdx * GPIO_NPINS) + pinIdx];
}

eSTATUS_t GPIOInitSPI (GPIOSPIInitConf_t conf) {

    GPIOBaseInit ();

    if (BUS_ID_IS_SPI (conf.busId) == FALSE) {
        return eSTATUS_FAILURE;
    }

    IO_t* pSCK  = GPIOGetIOfromId (conf.sckId);
    IO_t* pMISO = GPIOGetIOfromId (conf.misoId);
    IO_t* pMOSI = GPIOGetIOfromId (conf.mosiId);

    if (pSCK == NULL || pMISO == NULL || pMOSI == NULL) {
        return eSTATUS_FAILURE;
    }

    if (pSCK->ownerId != eDEVICE_ID_NULL || pMISO->ownerId != eDEVICE_ID_NULL ||
        pMOSI->ownerId != eDEVICE_ID_NULL) {
        return eSTATUS_FAILURE;
    }

    GPIOEnablePortClock (conf.sckId);
    GPIOEnablePortClock (conf.misoId);
    GPIOEnablePortClock (conf.mosiId);

    pSCK->ownerId  = conf.busId;
    pMISO->ownerId = conf.busId;
    pMOSI->ownerId = conf.busId;

    GPIO_InitTypeDef GPIO_InitStruct = { 0U };
    GPIO_InitStruct.Mode             = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin              = pSCK->pin;
    GPIO_InitStruct.Alternate        = conf.alternate;
    HAL_GPIO_Init (pSCK->pPort, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = pMISO->pin;
    HAL_GPIO_Init (pMISO->pPort, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = pMOSI->pin;
    HAL_GPIO_Init (pMOSI->pPort, &GPIO_InitStruct);

    /* Only configure NSS if its being used*/
    if (conf.nssId != eGPIO_ID_NULL) {

        IO_t* pNSS = GPIOGetIOfromId (conf.nssId);
        if (pNSS == NULL) {
            return eSTATUS_FAILURE;
        }

        if (pNSS->ownerId != eDEVICE_ID_NULL) {
            return eSTATUS_FAILURE;
        }

        GPIOEnablePortClock (conf.nssId);
        pNSS->ownerId = conf.busId;

        GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Pin       = pNSS->pin;
        GPIO_InitStruct.Alternate = conf.alternate;
        HAL_GPIO_Init (pNSS->pPort, &GPIO_InitStruct);
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t GPIOInitI2C (GPIOI2CInitConf_t conf) {

    GPIOBaseInit ();

    if (BUS_ID_IS_I2C (conf.busId) == FALSE) {
        return eSTATUS_FAILURE;
    }

    IO_t* pSCL = GPIOGetIOfromId (conf.sclId);
    IO_t* pSDA = GPIOGetIOfromId (conf.sdaId);
    if (pSCL == NULL || pSDA == NULL) {
        return eSTATUS_FAILURE;
    }

    if (pSCL->ownerId != eDEVICE_ID_NULL || pSDA->ownerId != eDEVICE_ID_NULL) {
        return eSTATUS_FAILURE;
    }

    GPIOEnablePortClock (conf.sclId);
    GPIOEnablePortClock (conf.sdaId);
    pSCL->ownerId = conf.busId;
    pSDA->ownerId = conf.busId;

    GPIO_InitTypeDef GPIO_InitStruct = { 0U };
    GPIO_InitStruct.Mode             = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin              = pSCL->pin;
    GPIO_InitStruct.Alternate        = conf.alternate;
    HAL_GPIO_Init (pSCL->pPort, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = pSDA->pin;
    HAL_GPIO_Init (pSDA->pPort, &GPIO_InitStruct);

    return eSTATUS_SUCCESS;
}

eSTATUS_t GPIOInitUART (GPIOUARTInitConf_t conf) {

    // static BOOL_t initialized = FALSE;
    // if (initialized == FALSE) {
    //     REG_UART (eUART_1_BUS_ID, eGPIO_PORTID_B | eGPIO_PINID_15,
    //     eGPIO_PORTID_B | eGPIO_PINID_14, GPIO_AF7_USART1); REG_UART
    //     (eUART_2_BUS_ID, eGPIO_PORTID_D | eGPIO_PINID_6, eGPIO_PORTID_D
    //     | eGPIO_PINID_5, GPIO_AF7_USART2); REG_UART (eUART_3_BUS_ID,
    //     eGPIO_PORTID_B | eGPIO_PINID_11, eGPIO_PORTID_B |
    //     eGPIO_PINID_10, GPIO_AF7_USART3); initialized = TRUE;
    // }

    GPIOBaseInit ();

    if (BUS_ID_IS_UART (conf.busId) == FALSE) {
        return eSTATUS_FAILURE;
    }

    IO_t* pTX = GPIOGetIOfromId (conf.txId);
    IO_t* pRX = GPIOGetIOfromId (conf.rxId);
    if (pTX == NULL || pRX == NULL) {
        return eSTATUS_FAILURE;
    }

    if (pTX->ownerId != eDEVICE_ID_NULL || pRX->ownerId != eDEVICE_ID_NULL) {
        return eSTATUS_FAILURE;
    }

    GPIOEnablePortClock (conf.txId);
    GPIOEnablePortClock (conf.rxId);
    pTX->ownerId = conf.busId;
    pRX->ownerId = conf.busId;

    GPIO_InitTypeDef GPIO_InitStruct = { 0U };
    GPIO_InitStruct.Mode             = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin              = pTX->pin;
    GPIO_InitStruct.Alternate        = conf.alternate;
    HAL_GPIO_Init (pTX->pPort, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = pRX->pin;
    HAL_GPIO_Init (pRX->pPort, &GPIO_InitStruct);

    return eSTATUS_SUCCESS;
}

eSTATUS_t GPIOInitIO (GPIOIOInitConf_t conf) {

    GPIOBaseInit ();

    IO_t* pIO = GPIOGetIOfromId (conf.gpioId);
    if (pIO == NULL) {
        return eSTATUS_FAILURE;
    }

    if (pIO->pPort == NULL) {
        return eSTATUS_FAILURE;
    }

    if (pIO->ownerId != eDEVICE_ID_NULL) {
        return eSTATUS_FAILURE;
    }

    GPIOEnablePortClock (conf.gpioId);
    pIO->ownerId = conf.ownerId;

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Mode             = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull             = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Pin              = pIO->pin;
    HAL_GPIO_Init (pIO->pPort, &GPIO_InitStruct);

    return eSTATUS_SUCCESS;
}