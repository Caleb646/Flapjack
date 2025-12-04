#ifndef DRIVERS_BUS_SPI_H
#define DRIVERS_BUS_SPI_H

#include <stdint.h>

#include "common.h"

#include "drivers/driver.h"

#include "drivers/bus/bus_defs.h"
#include "drivers/bus/spi_defs.h"

#include "drivers/io/gpio_defs.h"

#include "cfg/bus/bus.h"

#include "platform/platform.h"

DRIVER_DECLARE_ARRAY (SPIDevice_t, SPIDevices, PLAT_SPI_MAX_DEVS);

SPIDevice_t* Plat_SPI_Init (eSPI_DEV_ID_t devId, BusDeviceCfg_t* pCfg);
eSTATUS_t Plat_SPI_Write (SPIDevice_t* pDev, uint8_t const* pTx, uint32_t size);
eSTATUS_t Plat_SPI_Read (SPIDevice_t* pDev, uint8_t* pRx, uint32_t size);
eSTATUS_t Plat_SPI_WriteRead (SPIDevice_t* pDev, uint8_t const* pTx, uint8_t* pRx, uint32_t size);

eSTATUS_t SPI_Init (BusDeviceCfg_t* pCfg, BusDeviceSPI_t* pOutBusDeviceSpi);
eSTATUS_t SPI_Write (BusDeviceSPI_t* pBusDeviceSpi, uint8_t const* pTx, uint32_t size);
eSTATUS_t SPI_Read (BusDeviceSPI_t* pBusDeviceSpi, uint8_t* pRx, uint32_t size);
eSTATUS_t SPI_WriteRead (BusDeviceSPI_t* pBusDeviceSpi, uint8_t const* pTx, uint8_t* pRx, uint32_t size);
void SPI_Wait (BusDeviceSPI_t* pBusDeviceSpi);

#endif // DRIVERS_BUS_SPI_H