#include "peripheral/gpio.h"
#include "common.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "hal.h"
#include "log/logger.h"
#include "mem/mem.h"
#include <stdint.h>
#include <string.h>

#define GPIO_NPORTS eGPIO_PORTID_MAX
#define GPIO_NPINS  eGPIO_PINID_MAX
#define PORT_OFFSET 0x400 // each port is offset by 0x400

static SHARED_MEM_SECTION IO_t gIOs[GPIO_NPORTS * GPIO_NPINS] = { 0 };

static void GPIOEnablePortClock (eGPIO_ID_t gpioId) {

    vIO_t* pIO = GPIOGetIOfromId (gpioId);
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

static void GPIOSystemInit (void) {

    static bool initialized = false;
    if (initialized == true) {
        return;
    }
    for (uint32_t portId = 0; portId < GPIO_NPORTS; ++portId) {
        for (uint32_t pinId = 0; pinId < GPIO_NPINS; ++pinId) {
            vIO_t* pIO   = &gIOs[(portId * GPIO_NPINS) + pinId];
            pIO->pPort   = GPIOA + (portId * PORT_OFFSET);
            pIO->pin     = GPIO_PIN_0 << pinId;
            pIO->ownerId = eDEVICE_ID_NULL;
        }
    }
    initialized = true;
}

vIO_t* GPIOGetIOfromId (eGPIO_ID_t gpioId) {

    if (GPIO_ID_IS_GPIO (gpioId) == false) {
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

eSTATUS_t GPIOFreeById (eGPIO_ID_t gpioId) {
    return GPIOFreeByIO (GPIOGetIOfromId (gpioId));
}

eSTATUS_t GPIOFreeByIO (vIO_t* pIO) {
    if (pIO == NULL) {
        return eSTATUS_FAILURE;
    }
    pIO->ownerId = eDEVICE_ID_NULL;
    return eSTATUS_SUCCESS;
}

eSTATUS_t GPIOInit (eDEVICE_ID_t ownerId, GPIOBoardConf_t boardConf) {

    GPIOSystemInit ();

    GPIOSharedConf_t shared = boardConf.conf;
    if (boardConf.pShared != NULL) {
        shared = *(boardConf.pShared);
    }

    eGPIO_ID_t gpioId = boardConf.id;
    uint8_t mode      = shared.mode;
    uint8_t pull      = shared.pull;
    uint8_t speed     = shared.speed;
    uint8_t alternate = shared.alternate;

    vIO_t* pIO = GPIOGetIOfromId (gpioId);
    if (pIO == NULL) {
        return eSTATUS_FAILURE;
    }

    if (pIO->ownerId != eDEVICE_ID_NULL) {
        return eSTATUS_FAILURE;
    }

    GPIOEnablePortClock (gpioId);
    pIO->ownerId = ownerId;

    GPIO_InitTypeDef GPIO_InitStruct = { 0U };
    GPIO_InitStruct.Pin              = pIO->pin;
    GPIO_InitStruct.Mode             = mode;
    GPIO_InitStruct.Pull             = pull;
    GPIO_InitStruct.Speed            = speed;
    GPIO_InitStruct.Alternate        = alternate;
    HAL_GPIO_Init (pIO->pPort, &GPIO_InitStruct);

    return eSTATUS_SUCCESS;
}

eSTATUS_t GPIOInitSPI (GPIOSPIInitConf_t conf) {

    GPIOSystemInit ();

    if (BUS_ID_IS_SPI (conf.busId) == false) {
        return eSTATUS_FAILURE;
    }

    vIO_t* pSCK  = GPIOGetIOfromId (conf.sckId);
    vIO_t* pMISO = GPIOGetIOfromId (conf.misoId);
    vIO_t* pMOSI = GPIOGetIOfromId (conf.mosiId);

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

        vIO_t* pNSS = GPIOGetIOfromId (conf.nssId);
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

    GPIOSystemInit ();

    if (BUS_ID_IS_I2C (conf.busId) == false) {
        return eSTATUS_FAILURE;
    }

    vIO_t* pSCL = GPIOGetIOfromId (conf.sclId);
    vIO_t* pSDA = GPIOGetIOfromId (conf.sdaId);
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

    // static bool initialized = false;
    // if (initialized == false) {
    //     REG_UART (eUART_1_BUS_ID, eGPIO_PORTID_B | eGPIO_PINID_15,
    //     eGPIO_PORTID_B | eGPIO_PINID_14, GPIO_AF7_USART1); REG_UART
    //     (eUART_2_BUS_ID, eGPIO_PORTID_D | eGPIO_PINID_6, eGPIO_PORTID_D
    //     | eGPIO_PINID_5, GPIO_AF7_USART2); REG_UART (eUART_3_BUS_ID,
    //     eGPIO_PORTID_B | eGPIO_PINID_11, eGPIO_PORTID_B |
    //     eGPIO_PINID_10, GPIO_AF7_USART3); initialized = true;
    // }

    GPIOSystemInit ();

    if (BUS_ID_IS_UART (conf.busId) == false) {
        return eSTATUS_FAILURE;
    }

    vIO_t* pTX = GPIOGetIOfromId (conf.txId);
    vIO_t* pRX = GPIOGetIOfromId (conf.rxId);
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

    GPIOSystemInit ();

    vIO_t* pIO = GPIOGetIOfromId (conf.gpioId);
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