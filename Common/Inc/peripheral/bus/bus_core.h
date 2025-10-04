#ifndef PERIPHERAL_BUS_BUS_CORE_H
#define PERIPHERAL_BUS_BUS_CORE_H

#include <stdint.h>
#include <string.h>

typedef struct {
    uint8_t const* pTxData;
    uint8_t* pRxData;
    size_t txSize;
    size_t rxSize;
} BusTransaction_t;


#endif // PERIPHERAL_BUS_BUS_CORE_H