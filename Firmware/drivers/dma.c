#include "hal.h"
#include "target.h"

#include "core/core.h"

#include "drivers/dma.h"

#include "common/mem.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define MAX_DEVICES 2U
#define MAX_STREAMS 8U

#define PLAT_DEFINE(DEV_ID, STREAM_ID)                         \
    { .plat.Instance = DMA##DEV_ID##_Stream##STREAM_ID,        \
      .devId         = (DEV_ID),                               \
      .streamId      = (STREAM_ID),                            \
      .irqId         = DMA##DEV_ID##_Stream##STREAM_ID##_IRQn, \
      .inUse         = false }

FJ_DEFINE_SHARED (static DmaHandle_t, s_Handles[MAX_STREAMS]) = {
    [0] = PLAT_DEFINE (1, 0), [1] = PLAT_DEFINE (1, 1), [2] = PLAT_DEFINE (1, 2),
    [3] = PLAT_DEFINE (1, 3), [4] = PLAT_DEFINE (1, 4), [5] = PLAT_DEFINE (1, 5),
    [6] = PLAT_DEFINE (1, 6), [7] = PLAT_DEFINE (1, 7)
};


static void Dma_IrqHandler (DmaHandle_t* pHandle) {

    if (pHandle) {
        HAL_DMA_IRQHandler (&pHandle->plat);
    }
}

#define DMA_IRQ_HANDLER(DEV_ID, STREAM_ID)                     \
    void DMA##DEV_ID##_Stream##STREAM_ID##_IRQHandler (void) { \
        Dma_IrqHandler (&s_Handles[(STREAM_ID)]);              \
    }

DMA_IRQ_HANDLER (1, 0);
DMA_IRQ_HANDLER (1, 1);
DMA_IRQ_HANDLER (1, 2);
DMA_IRQ_HANDLER (1, 3);
DMA_IRQ_HANDLER (1, 4);
DMA_IRQ_HANDLER (1, 5);
DMA_IRQ_HANDLER (1, 6);
DMA_IRQ_HANDLER (1, 7);


DmaHandle_t* DmaResource_Alloc (void) {

    DmaHandle_t* pHandle = NULL;
    for (uint32_t i = 0; i < MAX_STREAMS; ++i) {
        if (!s_Handles[i].inUse) {
            pHandle = &s_Handles[i];
        }
    }
    if (pHandle) {
        pHandle->inUse = true;
    }
    return pHandle;
}

eSTATUS_t Dma_Init (DmaHandle_t* pHandle) {

    eSTATUS_t status = eSTATUS_SUCCESS;
    do {
        if (!pHandle) {
            status = eSTATUS_FAILURE;
            break;
        }
        if (pHandle->plat.Init.Direction == DMA_MEMORY_TO_PERIPH && pHandle->plat.Init.FIFOMode == DMA_FIFOMODE_ENABLE) {
            LOG_ERROR ("Direct mode (fifo disabled) has to be used for memory-to-peripheral transfers");
            status = eSTATUS_FAILURE;
            break;
        }
        __HAL_RCC_DMA1_CLK_ENABLE ();
        __HAL_RCC_DMA2_CLK_ENABLE ();
        HAL_DMA_DeInit (&pHandle->plat);

        if (HAL_DMA_Init (&pHandle->plat) != HAL_OK) {
            status = eSTATUS_FAILURE;
            break;
        }

        HAL_NVIC_SetPriority (pHandle->irqId, 10, 10);
        HAL_NVIC_EnableIRQ (pHandle->irqId);

    } while (0);

    return status;
}