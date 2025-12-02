#ifndef CFG_BUS_H
#define CFG_BUS_H

#include <stdint.h>

#include "drivers/bus/bus_defs.h"

typedef struct {
    uint8_t busId;
    uint8_t busType; // eBUS_TYPE_t
    uint8_t spiNssGpioId;
    uint8_t i2cAddress;
} BusDeviceCfg_t;

#endif // CFG_BUS_H