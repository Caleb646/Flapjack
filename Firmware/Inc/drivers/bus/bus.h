#ifndef DRIVERS_BUS_BUS_H
#define DRIVERS_BUS_BUS_H

#include <stdint.h>

#include "common.h"

#include "cfg/cfg.h"

#include "drivers/bus/bus_defs.h"

#define BUS_MAX_DEVICES 16U

FJ_DECLARE_SHARED (BusDevice_t*, e_pBusDevices[BUS_MAX_DEVICES]);

BusDevice_t* Bus_Init (BusDeviceCfg_t const* pCfg);
// eSTATUS_t Bus_Write (BusDevice_t* pBusDevice, uint8_t const* pTx, uint32_t size);
// eSTATUS_t Bus_Read (BusDevice_t* pBusDevice, uint8_t* pRx, uint32_t size);
// eSTATUS_t Bus_WriteRead (BusDevice_t* pBusDevice, uint8_t const* pTx, uint8_t* pRx, uint32_t
// size); void Bus_Wait (BusDevice_t* pBusDevice);

#endif