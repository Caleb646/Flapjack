#ifndef DRIVER_DMA_H
#define DRIVER_DMA_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "drivers/dma_defs.h"

#include "drivers/tim_defs.h"

#include "platform/platform.h"

FJ_DECLARE_SHARED (DmaDevice_t*, e_pDmaDevices[PLAT_DMA_MAX_DEVS]);

eSTATUS_t Plat_Dma_Init (DmaCfg_t const* pCfg, DmaDevice_t* pOutDmaDevice);
void Plat_Dma_SetIrqHandler (DmaDevice_t* pDmaDevice, DmaIrqHandler_fn fnIrqHandler, void* pCtx);
void Plat_Dma_SetDeviceTransferCfg (DmaDevice_t* pDmaDevice, uint32_t srcAddr, uint32_t dstAddr, size_t size);
void Plat_Dma_SetDeviceEnabled (DmaDevice_t* pDmaDevice, bool enable);
void Plat_Dma_SetInterruptsEnabled (DmaDevice_t* pDmaDevice, bool enable);

static inline DmaDevice_t* Dma_GetFreeDevice (void) {
    return Plat_Dma_GetFree ();
}

DmaDevice_t* Dma_Init (DmaCfg_t const* pCfg);

eSTATUS_t Dma_InitForMemToMem (DmaDevice_t* pOutDmaDevice);
eSTATUS_t Dma_InitForMemToGpio (eTIM_DEVICE_ID_t timDevId, DmaDevice_t* pOutDmaDevice);
eSTATUS_t Dma_StartTransfer (DmaDevice_t* pDmaDevice, uint32_t srcAddr, uint32_t dstAddr, size_t size);

static inline void Dma_SetDeviceEnabled (DmaDevice_t* pDmaDevice, bool enable) {
    Plat_Dma_SetDeviceEnabled (pDmaDevice, enable);
}

// clang-format off
static inline void Dma_SetIrqHandler (DmaDevice_t* pDmaDevice, DmaIrqHandler_fn fnIrqHandler, void* pCtx) {
    Plat_Dma_SetIrqHandler (pDmaDevice, fnIrqHandler, pCtx);
}
// clang-format on

#endif // DRIVER_DMA_H