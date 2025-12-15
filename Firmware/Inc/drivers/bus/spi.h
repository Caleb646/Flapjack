#ifndef DRIVERS_BUS_SPI_H
#define DRIVERS_BUS_SPI_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "drivers/driver.h"

#include "drivers/bus/bus_defs.h"
#include "drivers/bus/spi_defs.h"

#include "platform/platform.h"

DRIVER_DECLARE_ARRAY (SPIDevice_t, SPIDevices, PLAT_SPI_MAX_DEVS);

SPIDevice_t* Plat_SPI_Init (eSPI_DEV_ID_t devId);
eSTATUS_t Plat_SPI_WriteRead_Block (SPIDevice_t* pDev, uint8_t const* pTx, uint8_t* pRx, uint32_t size);
bool Plat_SPI_IsBusy (SPIDevice_t* pDev);

eSTATUS_t SPI_Init (BusDeviceCfg_t* pCfg, BusDeviceSPI_t* pOutBusDeviceSpi);
eSTATUS_t SPI_WriteRead_Block (BusDeviceSPI_t* pBusDeviceSpi, uint8_t const* pTx, uint8_t* pRx, uint32_t size);
eSTATUS_t SPI_WriteRegister (BusDeviceSPI_t* pBusDeviceSpi, uint8_t reg, uint8_t const* pData, uint32_t len);
eSTATUS_t SPI_ReadRegister (BusDeviceSPI_t* pBusDeviceSpi, uint8_t reg, uint8_t* pData, uint32_t len);
void SPI_Wait (BusDeviceSPI_t* pBusDeviceSpi);

#endif // DRIVERS_BUS_SPI_H