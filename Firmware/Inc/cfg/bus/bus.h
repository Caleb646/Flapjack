#ifndef CFG_BUS_H
#define CFG_BUS_H

#include <stdint.h>

#include "drivers/bus/bus_defs.h"

#include "drivers/io/gpio_defs.h"

typedef struct {
    eBUS_DEV_ID_t busId;
    eBUS_TYPE_t busType;
    eGPIO_ID_t spiNssGpioId;
    uint8_t i2cAddress;
} BusDeviceCfg_t;

#endif // CFG_BUS_H