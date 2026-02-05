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

#define PORT(...) I_GPIO_ID_MAKE_PORTID_EXPAND (__VA_ARGS__)
#define PIN(...)  GPIO_ID_MAKE_PINID (GET_SECOND (__VA_ARGS__))

FJ_DEFINE_SHARED (SpiDevice_t, e_SpiDevices[5]) = {
#if TARG_SPI_ENABLED(2)
    { .devId = eBUS_ID_SPI_2, .pInstance = NULL },
#endif
};

FJ_DEFINE_SHARED (SpiHardware_t, e_SpiHwCfgs[]) = {
#if TARG_SPI_ENABLED(2)
    {
    .devId      = eBUS_ID_SPI_2,
    .pInstance  = SPI2,
    .pRcc       = &(RCC->APB1LENR),
    .rccMask    = RCC_APB1LENR_SPI2EN,
    .sckGpioId  = GPIO_ID_MAKE (TARG_SPI_2_SCK),
    .misoGpioId = GPIO_ID_MAKE (TARG_SPI_2_MISO),
    .mosiGpioId = GPIO_ID_MAKE (TARG_SPI_2_MOSI),
#if PORT(TARG_SPI_2_SCK) == PORT(A) && PIN(TARG_SPI_2_SCK) == PIN(0, 12) && \
PORT(TARG_SPI_2_MISO) == PORT(C) && PIN(TARG_SPI_2_MISO) == PIN(0, 2) &&    \
PORT(TARG_SPI_2_MOSI) == PORT(C) && PIN(TARG_SPI_2_MOSI) == PIN(0, 3)
    .afId = GPIO_AF5_SPI2,
#else
#error Unsupported SPI2 pin configuration
#endif // SPI2 pins
    },
#endif
};

FJ_DEFINE_SHARED (uint8_t, e_nSpiDevices) = sizeof (e_SpiHwCfgs) / sizeof (SpiHardware_t);


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

SpiDevice_t* Plat_SPI_Init (eBUS_DEV_ID_t devId) {

    SpiDevice_t* pDev  = Spi_GetDevice (devId);
    SpiHardware_t* pHw = Spi_GetHwCfg (devId);
    if (!pDev || !pHw || !pHw->pInstance) {
        return NULL;
    }

    if (pDev->pInstance) {
        // already initialized
        return pDev;
    }

    // enable rcc spi clock
    *(pHw->pRcc) |= pHw->rccMask;
    GPIO_t* pSCK  = GPIO_Init (pHw->sckGpioId, devId, PLAT_GPIO_CFG_AF_PP_NOPULL, pHw->afId);
    GPIO_t* pMISO = GPIO_Init (pHw->misoGpioId, devId, PLAT_GPIO_CFG_AF_PP_NOPULL, pHw->afId);
    GPIO_t* pMOSI = GPIO_Init (pHw->mosiGpioId, devId, PLAT_GPIO_CFG_AF_PP_NOPULL, pHw->afId);
    if (!pSCK || !pMISO || !pMOSI) {
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

eSTATUS_t Plat_SPI_WriteRead_Block (SpiDevice_t* pDev, uint8_t const* pTx, uint8_t* pRx, uint32_t size) {

    if (!pDev || !pDev->pInstance) {
        return eSTATUS_FAIL;
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

bool Plat_SPI_IsBusy (SpiDevice_t* pDev) {

    if (!pDev || !pDev->pInstance) {
        return false;
    }
    return LL_SPI_IsActiveFlag_EOT (pDev->pInstance) == 0U;
}