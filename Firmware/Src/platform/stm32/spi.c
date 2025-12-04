#include <stdint.h>

#include "common.h"

#include "drivers/bus/bus.h"
#include "drivers/bus/bus_defs.h"

#include "drivers/bus/spi.h"
#include "drivers/bus/spi_defs.h"

#include "drivers/io/gpio.h"
#include "drivers/io/gpio_defs.h"

#include "platform/stm32/platform.h"

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

    [SPI_DEV_ID_TO_INDEX (eSPI_DEV_ID_2)] = { .pInstance = SPI2,
                                              .pRcc      = &(RCC->APB1LENR),
                                              .rccMask   = RCC_APB1LENR_SPI2EN,
                                              .sckOpts   = { PLAT_GPIO_ID_MAKE (A, 12) },
                                              .misoOpts  = { PLAT_GPIO_ID_MAKE (C, 2) },
                                              .mosiOpts  = { PLAT_GPIO_ID_MAKE (C, 3) },
                                              .afOpts    = { GPIO_AF5_SPI2 } }, // eSPI_DEV_ID_2
};

void SPI1_IRQHandler (void) {
    // TODO
}

void SPI2_IRQHandler (void) {
    // TODO
}

void SPI3_IRQHandler (void) {
    // TODO
}

void SPI4_IRQHandler (void) {
    // TODO
}

void SPI5_IRQHandler (void) {
    // TODO
}

SPIDevice_t* Plat_SPI_Init (eSPI_DEV_ID_t devId, BusDeviceCfg_t* pCfg) {

    SPIDevice_t* pDev  = SPIDevices_GetMutable (SPI_DEV_ID_TO_INDEX (devId));
    SPIHardware_t* pHw = &g_SPIHw[SPI_DEV_ID_TO_INDEX (devId)];
    if (!pDev || !pHw->pInstance) {
        return NULL;
    }

    if (pDev->pInstance) {
        // already initialized
        return pDev;
    }

    // invalid gpio ids are zero
    if (!pCfg->spiSckGpioId || !pCfg->spiMisoGpioId || !pCfg->spiMosiGpioId) {
        return NULL;
    }

    // enable rcc spi clock
    *(pHw->pRcc) |= pHw->rccMask;
    bool found = false;
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
            found = true;
            break;
        }
        // clang-format on
    }

    if (!found) {
        return NULL;
    }

    LL_SPI_InitTypeDef defaultInit = {
        .TransferDirection = LL_SPI_FULL_DUPLEX,
        .Mode              = LL_SPI_MODE_MASTER,
        .DataWidth         = LL_SPI_DATAWIDTH_8BIT,
        .NSS               = LL_SPI_NSS_SOFT,
        .BaudRate          = LL_SPI_BAUDRATEPRESCALER_DIV8,
        .BitOrder          = LL_SPI_MSB_FIRST,
        .CRCCalculation    = LL_SPI_CRCCALCULATION_DISABLE,
        .ClockPolarity     = LL_SPI_POLARITY_HIGH,
        .ClockPhase        = LL_SPI_PHASE_2EDGE,
    };

    LL_SPI_Disable (pDev->pInstance);
    LL_SPI_DeInit (pDev->pInstance);
    LL_SPI_EnableGPIOControl (pDev->pInstance);
    LL_SPI_SetFIFOThreshold (pDev->pInstance, LL_SPI_FIFO_TH_01DATA);
    LL_SPI_Init (pDev->pInstance, &defaultInit);

    pDev->devId     = devId;
    pDev->pInstance = pHw->pInstance;
    return pDev;
}

eSTATUS_t Plat_SPI_WriteRead_Block (SPIDevice_t* pDev, uint8_t const* pTx, uint8_t* pRx, uint32_t size) {

    if (!pDev || !pDev->pInstance) {
        return eSTATUS_NULL_ARG;
    }

    LL_SPI_SetTransferSize (pDev->pInstance, size);
    LL_SPI_Enable (pDev->pInstance);
    LL_SPI_StartMasterTransfer (pDev->pInstance);
    while (size) {

        while (!LL_SPI_IsActiveFlag_TXP (pDev->pInstance)) {
        }

        uint8_t b = pTx ? *(pTx++) : 0xFF;
        LL_SPI_TransmitData8 (pDev->pInstance, b);

        while (!LL_SPI_IsActiveFlag_RXP (pDev->pInstance)) {
        }

        b = LL_SPI_ReceiveData8 (pDev->pInstance);
        if (pRx) {
            *(pRx++) = b;
        }
        --size;
    }
    while (!LL_SPI_IsActiveFlag_EOT (pDev->pInstance)) {
    }
    LL_SPI_ClearFlag_TXTF (pDev->pInstance);
    LL_SPI_Disable (pDev->pInstance);
}

bool Plat_SPI_IsBusy (SPIDevice_t* pDev) {

    if (!pDev || !pDev->pInstance) {
        return false;
    }
    return LL_SPI_IsActiveFlag_EOT (pDev->pInstance) == 0U;
}