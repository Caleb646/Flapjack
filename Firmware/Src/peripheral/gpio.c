#include "peripheral/gpio.h"
#include "common.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define GPIO_NPORTS GPIO_MAX_PORTS
#define GPIO_NPINS  eGPIO_PINID_MAX
#define PORT_OFFSET 0x400 // each port is offset by 0x400

static SHARED_MEM_SECTION bool g_IsSystemInitialized           = false;
static SHARED_MEM_SECTION IO_t g_IOs[GPIO_NPORTS * GPIO_NPINS] = { 0 };

#ifdef UNIT_TEST

void GPIOGetIOs (IO_t** ppIOs, uint32_t* pCount) {
    *ppIOs  = g_IOs;
    *pCount = GPIO_NPORTS * GPIO_NPINS;
}

#endif // UNIT_TEST

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

eSTATUS_t GPIOSystemInit (void) {

    if (g_IsSystemInitialized) {
        return eSTATUS_SUCCESS;
    }
    for (uint32_t portId = 0; portId < GPIO_NPORTS; ++portId) {
        for (uint32_t pinId = 0; pinId < GPIO_NPINS; ++pinId) {
            vIO_t* pIO   = &g_IOs[(portId * GPIO_NPINS) + pinId];
            pIO->pPort   = (GPIO_TypeDef*)((uintptr_t)GPIOA + (portId * PORT_OFFSET));
            pIO->pin     = GPIO_PIN_0 << pinId;
            pIO->ownerId = eDEVICE_ID_NULL;
        }
    }
    g_IsSystemInitialized = true;
    return eSTATUS_SUCCESS;
}

vIO_t* GPIOGetIOfromId (eGPIO_ID_t gpioId) {

    if (!GPIO_ID_VALID (gpioId) || !g_IsSystemInitialized) {
        return NULL;
    }
    eGPIO_PORTID_t portIdx = GPIO_ID_TO_PORT_IDX (gpioId);
    eGPIO_PINID_t pinIdx   = GPIO_ID_TO_PIN_IDX (gpioId);
    uint32_t idx           = (portIdx * GPIO_NPINS) + pinIdx;
    if (idx >= (GPIO_NPORTS * GPIO_NPINS)) {
        return NULL;
    }
    return &g_IOs[(portIdx * GPIO_NPINS) + pinIdx];
}

eSTATUS_t GPIOFreeById (eGPIO_ID_t gpioId) {

    return GPIOFreeByIO (GPIOGetIOfromId (gpioId));
}

eSTATUS_t GPIOFreeByIO (vIO_t* pIO) {

    if (!GPIO_VALID (pIO) || !g_IsSystemInitialized) {
        return eSTATUS_FAILURE;
    }
    pIO->ownerId = eDEVICE_ID_NULL;
    return eSTATUS_SUCCESS;
}

eSTATUS_t GPIOInit (eDEVICE_ID_t ownerId, GPIODesc_t* pGPIODesc) {

    if (FJ_IS_NULL (pGPIODesc)) {
        return eSTATUS_NULL_ARG;
    }

    eGPIO_ID_t gpioId = GPIO_DESC_GET_ID (pGPIODesc);
    vIO_t* pIO        = GPIOGetIOfromId (gpioId);
    if (FJ_IS_NULL (pIO) || GPIO_HAS_OWNER (pIO)) {
        return eSTATUS_FAILURE;
    }

    GPIOEnablePortClock (gpioId);
    pIO->ownerId = ownerId;

    GPIO_InitTypeDef GPIO_InitStruct = { 0U };
    GPIO_InitStruct.Pin              = pIO->pin;
    GPIO_InitStruct.Mode             = GPIO_DESC_GET_MODE (pGPIODesc);
    GPIO_InitStruct.Pull             = GPIO_DESC_GET_PULL (pGPIODesc);
    GPIO_InitStruct.Speed            = GPIO_DESC_GET_SPEED (pGPIODesc);
    GPIO_InitStruct.Alternate        = GPIO_DESC_GET_ALTERNATE (pGPIODesc);
    HAL_GPIO_Init (pIO->pPort, &GPIO_InitStruct);

    return eSTATUS_SUCCESS;
}