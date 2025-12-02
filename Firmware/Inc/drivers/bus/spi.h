#ifndef DRIVERS_BUS_SPI_H
#define DRIVERS_BUS_SPI_H

#include <stdint.h>

#include "common.h"

#include "drivers/bus/bus_defs.h"
#include "drivers/bus/spi_defs.h"

#include "drivers/io/gpio_defs.h"

eSTATUS_t Plat_SPI_Init (eSPI_DEV_ID_t spiDevId, BusDeviceSPI_t* pOutSpiBusDevice);

eSTATUS_t SPI_Init (eSPI_DEV_ID_t spiDevId, BusDeviceSPI_t* pOutSpiBusDevice);


#endif // DRIVERS_BUS_SPI_H