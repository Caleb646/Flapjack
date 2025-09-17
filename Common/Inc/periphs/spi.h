#ifndef PERIPHS_SPI_H
#define PERIPHS_SPI_H

#include "common.h"
#include "hal.h"
#include <stdint.h>

typedef uint8_t eSPIBusId_t;
enum {
    eSPI_BUS_ID_1 = 0,
    eSPI_BUS_ID_2,
    eSPI_BUS_ID_3,
    eSPI_BUS_ID_4,
    eSPI_BUS_ID_5,
    eSPI_BUS_ID_MAX
};

typedef struct {
    eSPIBusId_t busId;

} SPIInitConf_t;

typedef struct {

} SPIBus_t;


#endif /* PERIPHS_SPI_H */