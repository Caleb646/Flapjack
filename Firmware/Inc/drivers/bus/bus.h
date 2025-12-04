#ifndef DRIVERS_BUS_BUS_H
#define DRIVERS_BUS_BUS_H

#include <stdint.h>

#include "common.h"

#include "drivers/bus/bus_defs.h"

#include "cfg/bus/bus.h"

// eSTATUS_t Plat_Bus_SPIInit (eSPI_DEV_ID_t spiDevId, BusDeviceSPI_t* pOutSpiBusDevice);

eSTATUS_t Bus_Init (BusDeviceCfg_t* pBusDeviceCfg, BusDevice_t* pOutBusDevice);
eSTATUS_t Bus_Write (BusDevice_t* pBusDevice, uint8_t const* pTx, uint32_t size);
eSTATUS_t Bus_Read (BusDevice_t* pBusDevice, uint8_t* pRx, uint32_t size);
eSTATUS_t Bus_WriteRead (BusDevice_t* pBusDevice, uint8_t const* pTx, uint8_t* pRx, uint32_t size);
void Bus_Wait (BusDevice_t* pBusDevice);

#endif