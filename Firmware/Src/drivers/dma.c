#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "core/core.h"

#include "drivers/dma.h"
#include "drivers/dma_defs.h"

#include "drivers/tim.h"
#include "drivers/tim_defs.h"

#include "platform/platform.h"

FJ_DEFINE_SHARED (DmaDevice_t*, e_pDmaDevices[PLAT_DMA_MAX_DEVS]);

DmaDevice_t* Dma_GetOrCreate (void) {

    DmaDevice_t* pDmaDevice = NULL;
    uint8_t index           = 0U;
    for (index = 0; index < PLAT_DMA_MAX_DEVS; ++index) {

        pDmaDevice = e_pDmaDevices[index];
        if (!pDmaDevice) {
            pDmaDevice = Alloc_SharedMem (sizeof (DmaDevice_t));
            if (!pDmaDevice) {
                return NULL;
            }
            e_pDmaDevices[index] = pDmaDevice;
            break;
        }

        if (pDmaDevice && !pDmaDevice->inUse) {
            break;
        }
    }

    if (pDmaDevice) {
        pDmaDevice->inUse  = true;
        pDmaDevice->devIdx = index;
    }
    return pDmaDevice;
}

DmaDevice_t* Dma_Init (DmaCfg_t const* pCfg) {

    DmaDevice_t* pDmaDevice = Dma_GetOrCreate ();
    if (!pDmaDevice) {
        return NULL;
    }

    if (Plat_Dma_Init (pCfg, pDmaDevice) != eSTATUS_OK) {
        goto error;
    }
    return pDmaDevice;
error:
    memset (pDmaDevice, 0, sizeof (DmaDevice_t));
    return NULL;
}

eSTATUS_t Dma_InitForMemToGpio (eTIM_DEVICE_ID_t timDevId, DmaDevice_t* pOutDmaDevice) {

    if (!TimDev_HasDmaSupport (timDevId)) {
        return eSTATUS_FAIL;
    }
    TimDmaReqMap_t* pDmaReqMap = TimDev_Get_DmaReqMap (timDevId);
    DmaCfg_t dmaCfg            = { 0 };
    dmaCfg.transferType        = eDMA_TRANSFER_TYPE_MEM_TO_PERIPH;
    dmaCfg.requestId           = pDmaReqMap->update;

    return Plat_Dma_Init (&dmaCfg, pOutDmaDevice);
}

eSTATUS_t Dma_StartTransfer (DmaDevice_t* pDmaDevice, uint32_t srcAddr, uint32_t dstAddr, size_t size) {

    if (!pDmaDevice || size == 0U) {
        return eSTATUS_FAIL;
    }

    Plat_Dma_SetDeviceEnabled (pDmaDevice, false);
    Plat_Dma_SetDeviceTransferCfg (pDmaDevice, srcAddr, dstAddr, size);
    Plat_Dma_SetInterruptsEnabled (pDmaDevice, true);
    Plat_Dma_SetDeviceEnabled (pDmaDevice, true);
    return eSTATUS_OK;
}
