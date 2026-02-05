#include <stdint.h>

#include "common.h"

#include "drivers/bus/bus.h"
#include "drivers/bus/bus_defs.h"
#include "drivers/bus/spi.h"
#include "drivers/bus/spi_defs.h"

#include "drivers/io/gpio.h"
#include "drivers/io/gpio_defs.h"

#include "platform/platform.h"

SpiHardware_t* Spi_GetHwCfg (eBUS_DEV_ID_t devId) {

    for (uint32_t i = 0; i < e_nSpiDevices; ++i) {
        SpiHardware_t* pHwCfg = &e_SpiHwCfgs[i];
        if (pHwCfg->devId == devId) {
            return pHwCfg;
        }
    }
    return NULL;
}

SpiDevice_t* Spi_GetDevice (eBUS_DEV_ID_t devId) {

    for (uint32_t i = 0; i < e_nSpiDevices; ++i) {
        SpiDevice_t* pDev = &e_SpiDevices[i];
        if (pDev->devId == devId) {
            return pDev;
        }
    }
    return NULL;
}

eSTATUS_t SPI_Init (BusDeviceCfg_t* pCfg, BusDeviceSPI_t* pOutBusDeviceSpi) {

    if (!pCfg || !pOutBusDeviceSpi) {
        return eSTATUS_FAIL;
    }

    SpiDevice_t* pDev = Plat_SPI_Init (pCfg->busId);
    if (!pDev) {
        return eSTATUS_FAIL;
    }

    GPIO_t* pNSS = GPIO_Init (pCfg->spiNssGpioId, pCfg->busId, PLAT_GPIO_CFG_OUT_PP_NOPULL, 0U);
    if (!pNSS) {
        return eSTATUS_FAIL;
    }
    pOutBusDeviceSpi->pNSS = pNSS;
    pOutBusDeviceSpi->pDev = pDev;
    GPIO_SetHigh (pOutBusDeviceSpi->pNSS);

    return eSTATUS_OK;
}

eSTATUS_t SPI_WriteRead_Block (BusDeviceSPI_t* pBusDeviceSpi, uint8_t const* pTx, uint8_t* pRx, uint32_t size) {

    if (!pBusDeviceSpi) {
        return eSTATUS_FAIL;
    }
    GPIO_SetLow (pBusDeviceSpi->pNSS);
    eSTATUS_t status = Plat_SPI_WriteRead_Block (pBusDeviceSpi->pDev, pTx, pRx, size);
    GPIO_SetHigh (pBusDeviceSpi->pNSS);
    return status;
}

eSTATUS_t SPI_WriteRegister (BusDeviceSPI_t* pBusDeviceSpi, uint8_t reg, uint8_t const* pData, uint32_t len) {

    if (!pBusDeviceSpi) {
        return eSTATUS_FAIL;
    }

    GPIO_SetLow (pBusDeviceSpi->pNSS);
    reg &= 0x7FU; // clear read bit
    eSTATUS_t status  = Plat_SPI_WriteRead_Block (pBusDeviceSpi->pDev, &reg, NULL, 1U);
    eSTATUS_t status2 = Plat_SPI_WriteRead_Block (pBusDeviceSpi->pDev, pData, NULL, len);
    GPIO_SetHigh (pBusDeviceSpi->pNSS);
    if (FJ_FAIL (status) || FJ_FAIL (status2)) {
        return eSTATUS_FAIL;
    }
    return eSTATUS_OK;
}

eSTATUS_t SPI_ReadRegister (BusDeviceSPI_t* pBusDeviceSpi, uint8_t reg, uint8_t* pData, uint32_t len) {

    if (!pBusDeviceSpi) {
        return eSTATUS_FAIL;
    }

    GPIO_SetLow (pBusDeviceSpi->pNSS);
    reg |= 0x80U; // set read bit
    eSTATUS_t status  = Plat_SPI_WriteRead_Block (pBusDeviceSpi->pDev, &reg, NULL, 1U);
    eSTATUS_t status2 = Plat_SPI_WriteRead_Block (pBusDeviceSpi->pDev, NULL, pData, len);
    GPIO_SetHigh (pBusDeviceSpi->pNSS);
    if (FJ_FAIL (status) || FJ_FAIL (status2)) {
        return eSTATUS_FAIL;
    }
    return eSTATUS_OK;
}

void SPI_Wait (BusDeviceSPI_t* pBusDeviceSpi) {

    if (!pBusDeviceSpi) {
        return;
    }

    while (Plat_SPI_IsBusy (pBusDeviceSpi->pDev)) {
        // wait
    }
}