#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "core/core.h"

#include "drivers/dma.h"
#include "drivers/dma_defs.h"

#include "drivers/tim.h"
#include "drivers/tim_defs.h"

#include "platform/platform.h"

eSTATUS_t Dma_InitForMemToMem (DmaDevice_t* pOutDmaDevice) {

    DmaCfg_t dmaCfg     = { 0 };
    dmaCfg.transferType = eDMA_TRANSFER_TYPE_MEM_TO_MEM;
    dmaCfg.requestId    = 0U;

    return Plat_Dma_Init (&dmaCfg, pOutDmaDevice);
}

eSTATUS_t Dma_InitForMemToGpio (eTIM_DEVICE_ID_t timDevId, DmaDevice_t* pOutDmaDevice) {

    if (!TimDev_HasDmaSupport (timDevId)) {
        return eSTATUS_FAILURE;
    }
    TimDmaReqMap_t* pDmaReqMap = TimDev_Get_DmaReqMap (timDevId);
    DmaCfg_t dmaCfg            = { 0 };
    dmaCfg.transferType        = eDMA_TRANSFER_TYPE_MEM_TO_PERIPH;
    dmaCfg.requestId           = pDmaReqMap->update;

    return Plat_Dma_Init (&dmaCfg, pOutDmaDevice);
}

eSTATUS_t Dma_StartTransfer (DmaDevice_t* pDmaDevice, uint32_t srcAddr, uint32_t dstAddr, size_t size) {

    if (!pDmaDevice || size == 0U) {
        return eSTATUS_NULL_ARG;
    }

    Plat_Dma_SetDeviceEnabled (pDmaDevice, false);
    Plat_Dma_SetDeviceTransferCfg (pDmaDevice, srcAddr, dstAddr, size);
    Plat_Dma_SetInterruptsEnabled (pDmaDevice, true);
    Plat_Dma_SetDeviceEnabled (pDmaDevice, true);
    return eSTATUS_SUCCESS;
}
