#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common.h"

#include "drivers/dma.h"
#include "drivers/dma_defs.h"

#include "platform/platform.h"

#include "targets/target.h"

#define DMA_MAX_DEVICES 8U

typedef struct DmaHwCfg_s {
    DMA_TypeDef* pDev;
    DMA_Stream_TypeDef* pStream;
    volatile uint32_t* pClkEnableReg;
    uint32_t clkEnableMsk;
} DmaHwCfg_t;

typedef struct DmaDevice_s {
    DMA_TypeDef* pDev;
    DMA_HandleTypeDef handle;
} DmaDevice_t;

static TARG_SHARED_MEM_SECTION DmaDevice_t* g_DmaDevices[DMA_MAX_DEVICES] = { 0 };
static TARG_SHARED_MEM_SECTION uint8_t g_DmaAllocated                     = 0U;

DmaHwCfg_t Stm32_Dma_GetHwCfg (uint32_t index) {

    DmaHwCfg_t dmaHwCfg = { 0 };

    switch (index) {
    case 0U: dmaHwCfg.pStream = DMA1_Stream0; break;
    case 1U: dmaHwCfg.pStream = DMA1_Stream1; break;
    case 2U: dmaHwCfg.pStream = DMA1_Stream2; break;
    case 3U: dmaHwCfg.pStream = DMA1_Stream3; break;
    case 4U: dmaHwCfg.pStream = DMA1_Stream4; break;
    case 5U: dmaHwCfg.pStream = DMA1_Stream5; break;
    default:
        // Invalid index
        return dmaHwCfg;
    }

    dmaHwCfg.pDev          = DMA1;
    dmaHwCfg.pClkEnableReg = &RCC->AHB1ENR;
    dmaHwCfg.clkEnableMsk  = RCC_AHB1ENR_DMA1EN;
    return dmaHwCfg;
}


DmaDevice_t* Plat_Dma_AllocDevice (DmaCfg_t const* pDmaCfg) {

    if (!pDmaCfg || g_DmaAllocated >= DMA_MAX_DEVICES) {
        return NULL;
    }

    DmaDevice_t** ppDmaDevice = &g_DmaDevices[g_DmaAllocated];

    if (!(*ppDmaDevice)) {
        *ppDmaDevice = Alloc_SharedMem (sizeof (DmaDevice_t));
    }

    if (!(*ppDmaDevice)) {
        return NULL;
    }

    DmaHwCfg_t hwCfg = Stm32_Dma_GetHwCfg (g_DmaAllocated);
    if (!hwCfg.pDev || !hwCfg.pStream) {
        return NULL;
    }

    *(hwCfg.pClkEnableReg) |= hwCfg.clkEnableMsk;
    // TODO: let dma cfg influence init params
    memset (*ppDmaDevice, 0, sizeof (DmaDevice_t));
    (*ppDmaDevice)->pDev                            = hwCfg.pDev;
    (*ppDmaDevice)->handle.Instance                 = hwCfg.pStream;
    (*ppDmaDevice)->handle.Init.Request             = DMA_REQUEST_MEM2MEM;
    (*ppDmaDevice)->handle.Init.Priority            = DMA_PRIORITY_HIGH;
    (*ppDmaDevice)->handle.Init.Direction           = DMA_MEMORY_TO_MEMORY;
    (*ppDmaDevice)->handle.Init.PeriphInc           = DMA_PINC_DISABLE;
    (*ppDmaDevice)->handle.Init.MemInc              = DMA_MINC_ENABLE;
    (*ppDmaDevice)->handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    (*ppDmaDevice)->handle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    (*ppDmaDevice)->handle.Init.Mode                = DMA_NORMAL;
    /*
     * When it is configured in direct mode ***(FIFO disabled)***, to
     * transfer data in memory-to-peripheral mode, the DMA preloads only
     * one data from the memory to the internal FIFO to ensure an immediate
     * data transfer as soon as a DMA request is triggered by a peripheral.
     */
    (*ppDmaDevice)->handle.Init.FIFOMode      = DMA_FIFOMODE_DISABLE;
    (*ppDmaDevice)->handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    (*ppDmaDevice)->handle.Init.MemBurst      = DMA_MBURST_SINGLE;
    (*ppDmaDevice)->handle.Init.PeriphBurst   = DMA_PBURST_SINGLE;

    if (HAL_DMA_Init (&((*ppDmaDevice)->handle)) != HAL_OK) {
        return NULL;
    }

    ++g_DmaAllocated;
    return *ppDmaDevice;
}

eSTATUS_t Plat_Dma_SetDeviceTransferCfg (DmaDevice_t* pDmaDevice, uint32_t srcAddr, uint32_t dstAddr, size_t size) {

    if (!pDmaDevice || !pDmaDevice->handle.Instance) {
        return eSTATUS_NULL_ARG;
    }

    /* Clear the DMAMUX synchro overrun flag */
    DMA_HandleTypeDef* hdma        = &(pDmaDevice->handle);
    hdma->DMAmuxChannelStatus->CFR = hdma->DMAmuxChannelStatusMask;
    if (hdma->DMAmuxRequestGen != 0U) {
        /* Clear the DMAMUX request generator overrun flag */
        hdma->DMAmuxRequestGenStatus->RGCFR = hdma->DMAmuxRequestGenStatusMask;
    }

    DMA_Base_Registers* regs_dma   = (DMA_Base_Registers*)hdma->StreamBaseAddress;
    BDMA_Base_Registers* regs_bdma = (BDMA_Base_Registers*)hdma->StreamBaseAddress;

    return 0;
}

void Plat_Dma_SetDeviceEnabled (DmaDevice_t* pDmaDevice, bool enable) {

    if (!pDmaDevice || !pDmaDevice->handle.Instance) {
        return;
    }

    if (enable) {
        __HAL_DMA_ENABLE (&(pDmaDevice->handle));
    } else {
        __HAL_DMA_DISABLE (&(pDmaDevice->handle));
    }
}