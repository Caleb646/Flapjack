#include <stdint.h>

#include "common.h"

#include "cfg/cfg.h"

#include "drivers/bus/bus.h"
#include "drivers/bus/bus_defs.h"
#include "drivers/bus/spi.h"
#include "drivers/bus/spi_defs.h"

#include "drivers/io/gpio.h"
#include "drivers/io/gpio_defs.h"

FJ_DEFINE_SHARED (BusDevice_t*, e_pBusDevices[BUS_MAX_DEVICES]);

FJ_TESTABLE BusDevice_t* Bus_FindDevice (BusDeviceCfg_t const* pCfg) {

    for (uint32_t i = 0; i < BUS_MAX_DEVICES; ++i) {
        BusDevice_t* pDev = e_pBusDevices[i];
        if (pDev && pDev->devId == pCfg->busId && pDev->busType == pCfg->busType) {
            return pDev;
        }
    }
    return NULL;
}

FJ_TESTABLE BusDevice_t* Bus_AllocDevice (void) {

    for (uint32_t i = 0; i < BUS_MAX_DEVICES; ++i) {
        if (!e_pBusDevices[i]) {
            BusDevice_t* pDev = (BusDevice_t*)Alloc_SharedMem (sizeof (BusDevice_t));
            if (!pDev) {
                return NULL;
            }
            e_pBusDevices[i] = pDev;
            return pDev;
        }
    }
    return NULL;
}

FJ_TESTABLE eSTATUS_t Bus_Init_ (BusDeviceCfg_t const* pCfg, BusDevice_t* pOutBusDev) {

    switch (pCfg->busType) {
    case eBUS_TYPE_SPI: return SPI_Init (pCfg, &pOutBusDev->spi);
    case eBUS_TYPE_I2C:
    default: return eSTATUS_FAIL;
    }
}

BusDevice_t* Bus_Init (BusDeviceCfg_t const* pCfg) {

    if (!pCfg) {
        return NULL;
    }

    BusDevice_t* pDev = Bus_FindDevice (pCfg);
    if (pDev) {
        return pDev;
    }

    pDev = Bus_AllocDevice ();
    if (!pDev) {
        return NULL;
    }

    if (FJ_FAIL (Bus_Init_ (pCfg, pDev))) {
        return NULL;
    }

    return pDev;
}

// eSTATUS_t Bus_Write (BusDevice_t* pBusDevice, uint8_t const* pTx, uint32_t size) {

//     if (!pBusDevice) {
//         return eSTATUS_FAIL;
//     }
//     switch (pBusDevice->busType) {
//     case eBUS_TYPE_SPI: return SPI_Write (&pBusDevice->spi, pTx, size);
//     case eBUS_TYPE_I2C: return eSTATUS_FAIL;
//     default: return eSTATUS_FAIL;
//     }
// }

// eSTATUS_t Bus_Read (BusDevice_t* pBusDevice, uint8_t* pRx, uint32_t size) {

//     if (!pBusDevice) {
//         return eSTATUS_FAIL;
//     }
//     switch (pBusDevice->busType) {
//     case eBUS_TYPE_SPI: return SPI_Read (&pBusDevice->spi, pRx, size);
//     case eBUS_TYPE_I2C: return eSTATUS_FAIL;
//     default: return eSTATUS_FAIL;
//     }
// }

// eSTATUS_t Bus_WriteRead (BusDevice_t* pBusDevice, uint8_t const* pTx, uint8_t* pRx, uint32_t size) {

//     if (!pBusDevice) {
//         return eSTATUS_FAIL;
//     }
//     switch (pBusDevice->busType) {
//     case eBUS_TYPE_SPI: return SPI_WriteRead (&pBusDevice->spi, pTx, pRx, size);
//     case eBUS_TYPE_I2C: return eSTATUS_FAIL;
//     default: return eSTATUS_FAIL;
//     }
// }

// void Bus_Wait (BusDevice_t* pBusDevice) {

//     if (!pBusDevice) {
//         return;
//     }
//     switch (pBusDevice->busType) {
//     case eBUS_TYPE_SPI: SPI_Wait (&pBusDevice->spi); break;
//     case eBUS_TYPE_I2C: break;
//     default: break;
//     }
// }