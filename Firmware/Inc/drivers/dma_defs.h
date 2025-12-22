#ifndef DRIVERS_DMA_DEFS_H
#define DRIVERS_DMA_DEFS_H

#include <stdbool.h>
#include <stdint.h>

#include "platform/platform.h"

typedef uint8_t eDMA_DIRECTION_t;
enum {
    eDMA_DIRECTION_MEM_TO_MEM = 0,
    eDMA_DIRECTION_MEM_TO_PERIPH,
    eDMA_DIRECTION_PERIPH_TO_MEM,
    eDMA_DIRECTION_PERIPH_TO_PERIPH,
};

typedef uint8_t eDMA_MODE_t;
enum {
    eDMA_MODE_NORMAL = 0,
    eDMA_MODE_CIRCULAR,
};

typedef struct DmaCfg_s {
    eDMA_DIRECTION_t direction;
    eDMA_MODE_t mode;
    uint32_t requestId;
} DmaCfg_t;

typedef struct DmaDevice_s DmaDevice_t;
typedef void (*DmaIrqHandler_fn) (DmaDevice_t* pDmaDevice, void* pCtx);

typedef struct DmaDevice_s {

    bool inUse;
    uint8_t devIdx;

    DMA_TypeDef* pDev;
    DMA_HandleTypeDef handle;

    DmaIrqHandler_fn fnIrqHandler;
    void* pCtx;

} DmaDevice_t;


#endif // DRIVERS_DMA_DEFS_H