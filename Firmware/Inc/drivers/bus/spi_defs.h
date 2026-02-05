#ifndef DRIVERS_BUS_SPI_DEFS_H
#define DRIVERS_BUS_SPI_DEFS_H

#include <stdint.h>

#include "drivers/bus/bus_defs.h"

#include "drivers/io/gpio_defs.h"

// typedef struct SPI_TypeDef;

typedef struct SpiDevice_s {
    eBUS_DEV_ID_t devId;
    SPI_TypeDef* pInstance;
} SpiDevice_t;

typedef struct SpiHardware_s {
    eBUS_DEV_ID_t devId;
    SPI_TypeDef* pInstance;
    uint32_t volatile* pRcc;
    uint32_t rccMask;
    eGPIO_ID_t sckGpioId;
    eGPIO_ID_t misoGpioId;
    eGPIO_ID_t mosiGpioId;
    uint32_t afId;
} SpiHardware_t;


#endif // DRIVERS_BUS_SPI_DEFS_H