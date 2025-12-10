#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "drivers/dma.h"
#include "drivers/dma_defs.h"

#include "platform/platform.h"


DmaDevice_t* Dma_AllocDeviceForMemToMem (void) {

    DmaCfg_t dmaCfg     = { 0 };
    dmaCfg.transferType = eDMA_TRANSFER_TYPE_MEM_TO_MEM;

    return Plat_Dma_AllocDevice (&dmaCfg);
}
