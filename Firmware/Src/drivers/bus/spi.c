#include <stdint.h>

#include "common.h"

#include "drivers/bus/bus.h"
#include "drivers/bus/bus_defs.h"
#include "drivers/bus/spi.h"
#include "drivers/bus/spi_defs.h"

#include "drivers/io/gpio.h"
#include "drivers/io/gpio_defs.h"

#include "platform/platform.h"

DRIVER_DEFINE_ARRAY (SPIDevice_t, SPIDevices, PLAT_SPI_MAX_DEVS);

eSTATUS_t SPI_Init (BusDeviceCfg_t* pCfg, BusDeviceSPI_t* pOutBusDeviceSpi) {

    if (!pCfg || !pOutBusDeviceSpi) {
        return eSTATUS_NULL_ARG;
    }

    SPIDevice_t* pDev = Plat_SPI_Init (pCfg->busId, pCfg);
    if (!pDev) {
        return eSTATUS_FAILURE;
    }

    GPIO_t* pNSS = GPIO_Init (pCfg->spiNssGpioId, pCfg->busId, PLAT_GPIO_CFG_OUT_PP_NOPULL, 0U);
    if (!pNSS) {
        return eSTATUS_FAILURE;
    }
    pOutBusDeviceSpi->pNSS = pNSS;
    pOutBusDeviceSpi->pDev = pDev;

    return eSTATUS_SUCCESS;
}

eSTATUS_t SPI_Write (BusDeviceSPI_t* pBusDeviceSpi, uint8_t const* pTx, uint32_t size) {

    if (!pBusDeviceSpi || !pBusDeviceSpi->pDev) {
        return eSTATUS_NULL_ARG;
    }
    return Plat_SPI_Write (pBusDeviceSpi->pDev, pTx, size);
}

eSTATUS_t SPI_Read (BusDeviceSPI_t* pBusDeviceSpi, uint8_t* pRx, uint32_t size) {

    if (!pBusDeviceSpi || !pBusDeviceSpi->pDev) {
        return eSTATUS_NULL_ARG;
    }
    return Plat_SPI_Read (pBusDeviceSpi->pDev, pRx, size);
}

eSTATUS_t SPI_WriteRead (BusDeviceSPI_t* pBusDeviceSpi, uint8_t const* pTx, uint8_t* pRx, uint32_t size) {

    if (!pBusDeviceSpi || !pBusDeviceSpi->pDev) {
        return eSTATUS_NULL_ARG;
    }
    return Plat_SPI_WriteRead (pBusDeviceSpi->pDev, pTx, pRx, size);
}

void SPI_Wait (BusDeviceSPI_t* pBusDeviceSpi) {
}