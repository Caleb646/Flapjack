#ifndef DRIVERS_DMA_DEFS_H
#define DRIVERS_DMA_DEFS_H

#include <stdbool.h>
#include <stdint.h>

// #include "platform/platform.h"

typedef uint8_t eDMA_TRANSFER_TYPE_t;
enum {
    eDMA_TRANSFER_TYPE_MEM_TO_MEM = 0,
    eDMA_TRANSFER_TYPE_MEM_TO_PERIPH,
    eDMA_TRANSFER_TYPE_PERIPH_TO_MEM,
    eDMA_TRANSFER_TYPE_PERIPH_TO_PERIPH,
};

typedef struct DmaCfg_s {
    uint32_t unused;
} DmaCfg_t;

typedef struct DmaDevice_s DmaDevice_t;


#endif // DRIVERS_DMA_DEFS_H