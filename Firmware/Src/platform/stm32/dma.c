#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common.h"

#include "drivers/dma.h"
#include "drivers/dma_defs.h"

#include "platform/platform.h"

#include "targets/target.h"

// typedef struct {
//     volatile uint32_t ISR; /*!< DMA interrupt status register */
//     volatile uint32_t Reserved0;
//     volatile uint32_t IFCR; /*!< DMA interrupt flag clear register */
// } DMA_Base_Registers;

typedef struct DmaHwCfg_s {
    DMA_TypeDef* pDev;
    DMA_Stream_TypeDef* pStream;
    volatile uint32_t* pClkEnableReg;
    uint32_t clkEnableMsk;
    uint8_t irqNum;
} DmaHwCfg_t;


void Stm32_Dma_IrqHandler (DmaDevice_t* pDmaDevice) {

    if (!pDmaDevice) {
        return;
    }

    // DMA_HandleTypeDef* hdma      = &pDmaDevice->handle;
    // DMA_Base_Registers* regs_dma = (DMA_Base_Registers*)hdma->StreamBaseAddress;
    // if (__HAL_DMA_GET_IT_SOURCE (hdma, DMA_IT_TE)) {
    //     /* Disable the transfer error interrupt */
    //     ((DMA_Stream_TypeDef*)hdma->Instance)->CR &= ~(DMA_IT_TE);
    //     /* Clear the transfer error flag */
    //     regs_dma->IFCR = DMA_FLAG_TEIF0_4 << (hdma->StreamIndex & 0x1FU);
    // }

    // if (__HAL_DMA_GET_IT_SOURCE (hdma, DMA_IT_TC)) {
    //     /* Disable all the transfer interrupts */
    //     ((DMA_Stream_TypeDef*)hdma->Instance)->CR &= ~(DMA_IT_TC | DMA_IT_TE | DMA_IT_DME);
    //     ((DMA_Stream_TypeDef*)hdma->Instance)->FCR &= ~(DMA_IT_FE);
    //     /* Clear the transfer complete flag */
    //     regs_dma->IFCR = DMA_FLAG_TCIF0_4 << (hdma->StreamIndex & 0x1FU);
    // }

    if (pDmaDevice->fnIrqHandler) {
        pDmaDevice->fnIrqHandler (pDmaDevice, pDmaDevice->pCtx);
    } else {
        HAL_DMA_IRQHandler (&pDmaDevice->handle);
    }
}

#define DMA_DEF_IRQ_HANDLER(DEV_ID, STREAM_ID)                 \
    void DMA##DEV_ID##_Stream##STREAM_ID##_IRQHandler (void) { \
        Stm32_Dma_IrqHandler (e_pDmaDevices[STREAM_ID]);       \
    }

DMA_DEF_IRQ_HANDLER (1, 0);
DMA_DEF_IRQ_HANDLER (1, 1);
DMA_DEF_IRQ_HANDLER (1, 2);
DMA_DEF_IRQ_HANDLER (1, 3);
DMA_DEF_IRQ_HANDLER (1, 4);
DMA_DEF_IRQ_HANDLER (1, 5);
DMA_DEF_IRQ_HANDLER (1, 6);
DMA_DEF_IRQ_HANDLER (1, 7);

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


eSTATUS_t Plat_Dma_Init (DmaCfg_t const* pCfg, DmaDevice_t* pOutDmaDevice) {

    if (!pCfg || !pOutDmaDevice) {
        return eSTATUS_FAIL;
    }

    DmaHwCfg_t hwCfg = Stm32_Dma_GetHwCfg (pOutDmaDevice->devIdx);
    if (!hwCfg.pDev || !hwCfg.pStream) {
        return eSTATUS_FAIL;
    }

    *(hwCfg.pClkEnableReg) |= hwCfg.clkEnableMsk;
    // TODO: let dma cfg influence init params
    pOutDmaDevice->pDev                            = hwCfg.pDev;
    pOutDmaDevice->handle.Instance                 = hwCfg.pStream;
    pOutDmaDevice->handle.Init.Request             = DMA_REQUEST_MEM2MEM;
    pOutDmaDevice->handle.Init.Priority            = DMA_PRIORITY_HIGH;
    pOutDmaDevice->handle.Init.Direction           = DMA_MEMORY_TO_MEMORY;
    pOutDmaDevice->handle.Init.PeriphInc           = DMA_PINC_DISABLE;
    pOutDmaDevice->handle.Init.MemInc              = DMA_MINC_ENABLE;
    pOutDmaDevice->handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    pOutDmaDevice->handle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    pOutDmaDevice->handle.Init.Mode                = DMA_NORMAL;
    /*
     * When it is configured in direct mode ***(FIFO disabled)***, to
     * transfer data in memory-to-peripheral mode, the DMA preloads only
     * one data from the memory to the internal FIFO to ensure an immediate
     * data transfer as soon as a DMA request is triggered by a peripheral.
     */
    pOutDmaDevice->handle.Init.FIFOMode      = DMA_FIFOMODE_ENABLE;
    pOutDmaDevice->handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    pOutDmaDevice->handle.Init.MemBurst      = DMA_MBURST_SINGLE;
    pOutDmaDevice->handle.Init.PeriphBurst   = DMA_PBURST_SINGLE;

    if (pCfg->requestId != 0U) {
        pOutDmaDevice->handle.Init.Request = pCfg->requestId;
        // direct mode for peripheral transfers
        pOutDmaDevice->handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

        switch (pCfg->direction) {
        case eDMA_DIRECTION_MEM_TO_PERIPH:
            pOutDmaDevice->handle.Init.Direction = DMA_MEMORY_TO_PERIPH;
            break;
        case eDMA_DIRECTION_PERIPH_TO_MEM:
            pOutDmaDevice->handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
            break;
        default: return eSTATUS_FAIL;
        }
    }

    if (HAL_DMA_DeInit (&(pOutDmaDevice->handle)) != HAL_OK) {
        return eSTATUS_FAIL;
    }

    if (HAL_DMA_Init (&(pOutDmaDevice->handle)) != HAL_OK) {
        return eSTATUS_FAIL;
    }

    // TODO: let dma cfg influence irq priority
    HAL_NVIC_SetPriority (hwCfg.irqNum, 8, 8);
    HAL_NVIC_EnableIRQ (hwCfg.irqNum);

    return eSTATUS_OK;
}

// void Plat_Dma_SetTransferCfg (DmaDevice_t* pDmaDevice, uint32_t srcAddr, uint32_t dstAddr, size_t size) {

//     if (!pDmaDevice || !pDmaDevice->handle.Instance) {
//         return;
//     }

//     /* Clear the DMAMUX synchro overrun flag */
//     DMA_HandleTypeDef* hdma        = &(pDmaDevice->handle);
//     hdma->DMAmuxChannelStatus->CFR = hdma->DMAmuxChannelStatusMask;
//     if (hdma->DMAmuxRequestGen != 0U) {
//         /* Clear the DMAMUX request generator overrun flag */
//         hdma->DMAmuxRequestGenStatus->RGCFR = hdma->DMAmuxRequestGenStatusMask;
//     }

//     DMA_Base_Registers* regs_dma = (DMA_Base_Registers*)hdma->StreamBaseAddress;
//     /* Clear all interrupt flags at correct offset within the register */
//     regs_dma->IFCR = 0x3FUL << (hdma->StreamIndex & 0x1FU);
//     /* Clear DBM bit */
//     ((DMA_Stream_TypeDef*)hdma->Instance)->CR &= (uint32_t)(~DMA_SxCR_DBM);
//     /* Configure DMA Stream data length */
//     ((DMA_Stream_TypeDef*)hdma->Instance)->NDTR = size;
//     /* Peripheral to Memory */
//     if ((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH) {
//         /* Configure DMA Stream destination address */
//         ((DMA_Stream_TypeDef*)hdma->Instance)->PAR = dstAddr;
//         /* Configure DMA Stream source address */
//         ((DMA_Stream_TypeDef*)hdma->Instance)->M0AR = srcAddr;
//     }
//     /* Memory to Peripheral */
//     else {
//         /* Configure DMA Stream source address */
//         ((DMA_Stream_TypeDef*)hdma->Instance)->PAR = srcAddr;
//         /* Configure DMA Stream destination address */
//         ((DMA_Stream_TypeDef*)hdma->Instance)->M0AR = dstAddr;
//     }
// }

// void Plat_Dma_SetEnabled (DmaDevice_t* pDmaDevice, bool enable) {

//     if (!pDmaDevice || !pDmaDevice->handle.Instance) {
//         return;
//     }

//     if (enable) {
//         __HAL_DMA_ENABLE (&(pDmaDevice->handle));
//     } else {
//         __HAL_DMA_DISABLE (&(pDmaDevice->handle));
//     }
// }

// void Plat_Dma_SetInterruptsEnabled (DmaDevice_t* pDmaDevice, bool enable) {

//     if (!pDmaDevice || !pDmaDevice->handle.Instance) {
//         return;
//     }

//     uint32_t flags = DMA_IT_TC; // Transfer Complete
//     // flags |= DMA_IT_HT;         // Half Transfer
//     flags |= DMA_IT_TE; // Transfer Error
//     // flags |= DMA_IT_DME;        // Direct Mode Error
//     // flags |= DMA_IT_FE;         // FIFO Error

//     if (enable) {
//         __HAL_DMA_ENABLE_IT (&(pDmaDevice->handle), flags);
//     } else {
//         __HAL_DMA_DISABLE_IT (&(pDmaDevice->handle), flags);
//     }
// }