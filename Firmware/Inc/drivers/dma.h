#ifndef DRIVER_DMA_H
#define DRIVER_DMA_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "drivers/dma_defs.h"

DmaDevice_t* Plat_Dma_AllocDevice (DmaCfg_t const* pDmaCfg);
void Plat_Dma_SetDeviceEnabled (DmaDevice_t* pDmaDevice, bool enable);


DmaDevice_t* Dma_AllocDeviceForMemToMem (void);

#endif // DRIVER_DMA_H