#ifndef DRIVERS_DMA_H
#define DRIVERS_DMA_H

#include "hal.h"
#include "target.h"

#include "core/core.h"

#include <stdint.h>

typedef struct DmaHandle_s {
    DMA_HandleTypeDef plat;
    struct {
        uint8_t devId : 4;
        uint8_t streamId : 4;
    };
    uint16_t irqId;
    bool inUse;
} DmaHandle_t;

DmaHandle_t* DmaResource_Alloc (void);
eSTATUS_t Dma_Init (DmaHandle_t* pHandle);

#endif /* DRIVERS_DMA_H */