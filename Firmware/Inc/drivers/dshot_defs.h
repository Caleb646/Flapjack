#ifndef DRIVERS_DSHOT_DEFS_H
#define DRIVERS_DSHOT_DEFS_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "drivers/io/gpio_defs.h"

#include "targets/target.h"

// typedef struct DmaDevice_s {
//     uint8_t channel;
//     uint8_t stream;
//     uint32_t request;
// } DmaDevice_t;

typedef struct Dshot_s {
    GPIO_t* pports;
    uint32_t* pDmaBuffers;
    uint8_t portsInUse;
    uint8_t clrBitPos[TARG_MAX_MOTORS];
    uint8_t setBitPos[TARG_MAX_MOTORS];
} Dshot_t;

#endif // DRIVERS_DSHOT_DEFS_H