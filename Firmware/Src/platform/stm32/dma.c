#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common.h"

#include "drivers/dma.h"
#include "drivers/dma_defs.h"

#include "platform/platform.h"

#include "targets/target.h"

#define DMA_MAX_DEVICES 8U

typedef struct {
    __IO uint32_t ISR; /*!< DMA interrupt status register */
    __IO uint32_t Reserved0;
    __IO uint32_t IFCR; /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;

typedef struct DmaHwCfg_s {
    DMA_TypeDef* pDev;
    DMA_Stream_TypeDef* pStream;
    volatile uint32_t* pClkEnableReg;
    uint32_t clkEnableMsk;
    uint8_t irqNum;
} DmaHwCfg_t;

typedef struct DmaDevice_s DmaDevice_t;
typedef struct DmaDevice_s {
    DMA_TypeDef* pDev;
    DMA_HandleTypeDef handle;
    void* pCtx;
    void (*fnIrqHandler) (DmaDevice_t* pDmaDevice, void* pCtx);
} DmaDevice_t;

static TARG_SHARED_MEM_SECTION DmaDevice_t* g_DmaDevices[DMA_MAX_DEVICES] = { 0 };
static TARG_SHARED_MEM_SECTION uint8_t g_DmaAllocated                     = 0U;

void Stm32_Dma_IRQHandler (DmaDevice_t* pDmaDevice) {

    if (!pDmaDevice) {
        return;
    }

    DMA_HandleTypeDef* hdma      = &pDmaDevice->handle;
    DMA_Base_Registers* regs_dma = (DMA_Base_Registers*)hdma->StreamBaseAddress;
    if (__HAL_DMA_GET_IT_SOURCE (hdma, DMA_IT_TE)) {
        /* Disable the transfer error interrupt */
        ((DMA_Stream_TypeDef*)hdma->Instance)->CR &= ~(DMA_IT_TE);
        /* Clear the transfer error flag */
        regs_dma->IFCR = DMA_FLAG_TEIF0_4 << (hdma->StreamIndex & 0x1FU);
    }

    if (__HAL_DMA_GET_IT_SOURCE (hdma, DMA_IT_TC)) {
        /* Disable all the transfer interrupts */
        ((DMA_Stream_TypeDef*)hdma->Instance)->CR &= ~(DMA_IT_TC | DMA_IT_TE | DMA_IT_DME);
        ((DMA_Stream_TypeDef*)hdma->Instance)->FCR &= ~(DMA_IT_FE);
        /* Clear the transfer complete flag */
        regs_dma->IFCR = DMA_FLAG_TCIF0_4 << (hdma->StreamIndex & 0x1FU);
    }

    if (pDmaDevice->fnIrqHandler) {
        pDmaDevice->fnIrqHandler (pDmaDevice, pDmaDevice->pCtx);
    }
}

#define DMA_MAKE_IRQ_HANDLER(DEV_INDEX, STREAM_ID)                \
    void DMA##DEV_INDEX##_Stream##STREAM_ID##_IRQHandler (void) { \
        Stm32_Dma_IRQHandler (g_DmaDevices[STREAM_ID]);           \
    }

DMA_MAKE_IRQ_HANDLER (0, 0);
DMA_MAKE_IRQ_HANDLER (0, 1);
DMA_MAKE_IRQ_HANDLER (0, 2);
DMA_MAKE_IRQ_HANDLER (0, 3);
DMA_MAKE_IRQ_HANDLER (0, 4);
DMA_MAKE_IRQ_HANDLER (0, 5);
DMA_MAKE_IRQ_HANDLER (0, 6);
DMA_MAKE_IRQ_HANDLER (0, 7);

DmaHwCfg_t Stm32_Dma_GetHwCfg (uint32_t index) {

    DmaHwCfg_t dmaHwCfg = { 0 };

    switch (index) {
    case 0U:
        dmaHwCfg.pStream = DMA1_Stream0;
        dmaHwCfg.irqNum  = DMA1_Stream0_IRQn;
        break;
    case 1U:
        dmaHwCfg.pStream = DMA1_Stream1;
        dmaHwCfg.irqNum  = DMA1_Stream1_IRQn;
        break;
    case 2U:
        dmaHwCfg.pStream = DMA1_Stream2;
        dmaHwCfg.irqNum  = DMA1_Stream2_IRQn;
        break;
    case 3U:
        dmaHwCfg.pStream = DMA1_Stream3;
        dmaHwCfg.irqNum  = DMA1_Stream3_IRQn;
        break;
    case 4U:
        dmaHwCfg.pStream = DMA1_Stream4;
        dmaHwCfg.irqNum  = DMA1_Stream4_IRQn;
        break;
    case 5U:
        dmaHwCfg.pStream = DMA1_Stream5;
        dmaHwCfg.irqNum  = DMA1_Stream5_IRQn;
        break;
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

    // TODO: let dma cfg influence irq priority
    HAL_NVIC_SetPriority (hwCfg.irqNum, 8, 8);
    HAL_NVIC_EnableIRQ (hwCfg.irqNum);

    ++g_DmaAllocated;
    return *ppDmaDevice;
}

eSTATUS_t
Plat_Dma_RegisterCallback (DmaDevice_t* pDmaDevice, void (*fnIrqHandler) (DmaDevice_t* pDmaDevice, void* pCtx), void* pCtx) {

    if (!pDmaDevice) {
        return eSTATUS_NULL_ARG;
    }

    pDmaDevice->fnIrqHandler = fnIrqHandler;
    pDmaDevice->pCtx         = pCtx;
    return eSTATUS_SUCCESS;
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

    DMA_Base_Registers* regs_dma = (DMA_Base_Registers*)hdma->StreamBaseAddress;
    /* Clear all interrupt flags at correct offset within the register */
    regs_dma->IFCR = 0x3FUL << (hdma->StreamIndex & 0x1FU);
    /* Clear DBM bit */
    ((DMA_Stream_TypeDef*)hdma->Instance)->CR &= (uint32_t)(~DMA_SxCR_DBM);
    /* Configure DMA Stream data length */
    ((DMA_Stream_TypeDef*)hdma->Instance)->NDTR = size;
    /* Peripheral to Memory */
    if ((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH) {
        /* Configure DMA Stream destination address */
        ((DMA_Stream_TypeDef*)hdma->Instance)->PAR = dstAddr;
        /* Configure DMA Stream source address */
        ((DMA_Stream_TypeDef*)hdma->Instance)->M0AR = srcAddr;
    }
    /* Memory to Peripheral */
    else {
        /* Configure DMA Stream source address */
        ((DMA_Stream_TypeDef*)hdma->Instance)->PAR = srcAddr;
        /* Configure DMA Stream destination address */
        ((DMA_Stream_TypeDef*)hdma->Instance)->M0AR = dstAddr;
    }

    return eSTATUS_SUCCESS;
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

void Plat_Dma_SetInterruptsEnabled (DmaDevice_t* pDmaDevice, bool enable) {

    if (!pDmaDevice || !pDmaDevice->handle.Instance) {
        return;
    }

    uint32_t flags = DMA_IT_TC; // Transfer Complete
    // flags |= DMA_IT_HT;         // Half Transfer
    flags |= DMA_IT_TE; // Transfer Error
    // flags |= DMA_IT_DME;        // Direct Mode Error
    // flags |= DMA_IT_FE;         // FIFO Error

    if (enable) {
        __HAL_DMA_ENABLE_IT (&(pDmaDevice->handle), flags);
    } else {
        __HAL_DMA_DISABLE_IT (&(pDmaDevice->handle), flags);
    }
}