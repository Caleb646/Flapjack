#include <stdint.h>

#include "common.h"

#include "drivers/bus/bus.h"
#include "drivers/bus/bus_defs.h"

#include "drivers/bus/spi.h"
#include "drivers/bus/spi_defs.h"

#include "drivers/io/gpio.h"
#include "drivers/io/gpio_defs.h"

#include "platform/platform.h"

#include "targets/target.h"

// __HAL_RCC_SPI5_CLK_ENABLE ();

typedef struct SPIHardware_s {
    SPI_TypeDef* pInstance;
    uint32_t volatile* pRcc;
    uint32_t rccMask;
    eGPIO_ID_t sckOpts[PLAT_SPI_MAX_PIN_SEL];
    eGPIO_ID_t misoOpts[PLAT_SPI_MAX_PIN_SEL];
    eGPIO_ID_t mosiOpts[PLAT_SPI_MAX_PIN_SEL];
    uint8_t afOpts[PLAT_SPI_MAX_PIN_SEL];
} SPIHardware_t;

static SPIHardware_t g_SPIHw[PLAT_SPI_MAX_DEVS] = {

    [eSPI_DEV_ID_2] = { .pInstance = SPI2,
                        .pRcc      = &(RCC->APB1LENR),
                        .rccMask   = RCC_APB1LENR_SPI2EN,
                        .sckOpts   = { GPIO_ID_MAKE (A, 12) },
                        .misoOpts  = { GPIO_ID_MAKE (C, 2) },
                        .mosiOpts  = { GPIO_ID_MAKE (C, 3) },
                        .afOpts    = { GPIO_AF5_SPI2 } }, // eSPI_DEV_ID_2
};

SPIDevice_t* Plat_SPI_Init (eSPI_DEV_ID_t devId, BusDeviceCfg_t* pCfg) {

    SPIDevice_t* pDev  = SPIDevices_GetMutable (SPI_DEV_ID_TO_INDEX (devId));
    SPIHardware_t* pHw = &g_SPIHw[SPI_DEV_ID_TO_INDEX (devId)];
    if (!pDev) {
        return NULL;
    }

    if (pDev->pInstance) {
        // Already initialized
        return pDev;
    }

    // invalid gpio ids are zero
    if (!pCfg->spiSckGpioId || !pCfg->spiMisoGpioId || !pCfg->spiMosiGpioId) {
        return NULL;
    }

    // enable rcc spi clock
    *(pHw->pRcc) |= pHw->rccMask;
    for (uint32_t i = 0; i < PLAT_SPI_MAX_PIN_SEL; ++i) {
        // clang-format off
        if (pCfg->spiSckGpioId == pHw->sckOpts[i] &&
            pCfg->spiMisoGpioId == pHw->misoOpts[i] &&
            pCfg->spiMosiGpioId == pHw->mosiOpts[i]) {
            GPIO_t* pSCK  = GPIO_Init (pCfg->spiSckGpioId, pCfg->busId, PLAT_GPIO_CFG_AF_PP_NOPULL, pHw->afOpts[i]);
            GPIO_t* pMISO = GPIO_Init (pCfg->spiMisoGpioId, pCfg->busId, PLAT_GPIO_CFG_AF_PP_NOPULL, pHw->afOpts[i]);
            GPIO_t* pMOSI = GPIO_Init (pCfg->spiMosiGpioId, pCfg->busId, PLAT_GPIO_CFG_AF_PP_NOPULL, pHw->afOpts[i]);
            if (!pSCK || !pMISO || !pMOSI) {
                return NULL;
            }
            break;
        }
        // clang-format on
    }

    pDev->devId     = devId;
    pDev->pInstance = pHw->pInstance;
    return pDev;
}

eSTATUS_t Plat_SPI_Write (SPIDevice_t* pDev, uint8_t const* pTx, uint32_t size) {
}

eSTATUS_t Plat_SPI_Read (SPIDevice_t* pDev, uint8_t* pRx, uint32_t size) {
}

eSTATUS_t Plat_SPI_WriteRead (SPIDevice_t* pDev, uint8_t const* pTx, uint8_t* pRx, uint32_t size) {
}