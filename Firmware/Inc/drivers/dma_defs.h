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
    eDMA_TRANSFER_TYPE_t transferType;
    uint32_t requestId;
} DmaCfg_t;

typedef struct DmaDevice_s DmaDevice_t;

typedef void (*fnDmaIrqHandler_t) (DmaDevice_t* pDmaDevice, void* pCtx);


#endif // DRIVERS_DMA_DEFS_H