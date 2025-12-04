#include <stdint.h>

#include "common.h"

#include "drivers/bus/bus.h"
#include "drivers/bus/bus_defs.h"
#include "drivers/bus/spi.h"
#include "drivers/bus/spi_defs.h"

#include "drivers/io/gpio.h"
#include "drivers/io/gpio_defs.h"

eSTATUS_t Bus_Init (BusDeviceCfg_t* pBusDeviceCfg, BusDevice_t* pOutBusDevice) {

    if (!pBusDeviceCfg || !pOutBusDevice) {
        return eSTATUS_NULL_ARG;
    }

    switch (pBusDeviceCfg->busType) {
    case eBUS_TYPE_SPI: return SPI_Init (pBusDeviceCfg, &pOutBusDevice->spi);
    case eBUS_TYPE_I2C: return eSTATUS_NOT_SUPPORTED;
    default: return eSTATUS_INVALID_ARG;
    }
}

eSTATUS_t Bus_Write (BusDevice_t* pBusDevice, uint8_t const* pTx, uint32_t size) {

    if (!pBusDevice) {
        return eSTATUS_NULL_ARG;
    }
    switch (pBusDevice->busType) {
    case eBUS_TYPE_SPI: return SPI_Write (&pBusDevice->spi, pTx, size);
    case eBUS_TYPE_I2C: return eSTATUS_NOT_SUPPORTED;
    default: return eSTATUS_INVALID_ARG;
    }
}

eSTATUS_t Bus_Read (BusDevice_t* pBusDevice, uint8_t* pRx, uint32_t size) {

    if (!pBusDevice) {
        return eSTATUS_NULL_ARG;
    }
    switch (pBusDevice->busType) {
    case eBUS_TYPE_SPI: return SPI_Read (&pBusDevice->spi, pRx, size);
    case eBUS_TYPE_I2C: return eSTATUS_NOT_SUPPORTED;
    default: return eSTATUS_INVALID_ARG;
    }
}

eSTATUS_t Bus_WriteRead (BusDevice_t* pBusDevice, uint8_t const* pTx, uint8_t* pRx, uint32_t size) {

    if (!pBusDevice) {
        return eSTATUS_NULL_ARG;
    }
    switch (pBusDevice->busType) {
    case eBUS_TYPE_SPI: return SPI_WriteRead (&pBusDevice->spi, pTx, pRx, size);
    case eBUS_TYPE_I2C: return eSTATUS_NOT_SUPPORTED;
    default: return eSTATUS_INVALID_ARG;
    }
}

void Bus_Wait (BusDevice_t* pBusDevice) {

    if (!pBusDevice) {
        return;
    }
    switch (pBusDevice->busType) {
    case eBUS_TYPE_SPI: SPI_Wait (&pBusDevice->spi); break;
    case eBUS_TYPE_I2C: break;
    default: break;
    }
}