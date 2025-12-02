#ifndef DRIVERS_BUS_BUS_H
#define DRIVERS_BUS_BUS_H

#include <stdint.h>

#include "common.h"

#include "drivers/bus/bus_defs.h"

#include "cfg/bus/bus.h"

// eSTATUS_t Plat_Bus_SPIInit (eSPI_DEV_ID_t spiDevId, BusDeviceSPI_t* pOutSpiBusDevice);

eSTATUS_t Bus_Init (BusDeviceCfg_t busDeviceCfg, BusDevice_t* pOutBusDevice);

#endif