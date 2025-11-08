#include "peripheral/dma.h"
#include "core/core.h"
#include "core/log/logger.h"
#include "hal.h"
#include "mem/mem.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>


static SHARED_MEM_SECTION DMAStream_t gDMAStreams[eDMA_STREAM_MAX] = { 0 };

/**
 * @brief This function handles DMA1 stream0 global interrupt.
 */
void DMA1_Stream0_IRQHandler (void) {
    HAL_DMA_IRQHandler (&DMAGetStreamById (eDMA_STREAM_0)->handle);
}

/**
 * @brief This function handles DMA1 stream1 global interrupt.
 */
void DMA1_Stream1_IRQHandler (void) {
    HAL_DMA_IRQHandler (&DMAGetStreamById (eDMA_STREAM_1)->handle);
}

/**
 * @brief This function handles DMA1 stream2 global interrupt.
 */
void DMA1_Stream2_IRQHandler (void) {
    HAL_DMA_IRQHandler (&DMAGetStreamById (eDMA_STREAM_2)->handle);
}

/**
 * @brief This function handles DMA1 stream3 global interrupt.
 */
void DMA1_Stream3_IRQHandler (void) {
    HAL_DMA_IRQHandler (&DMAGetStreamById (eDMA_STREAM_3)->handle);
}

/**
 * @brief This function handles DMA1 stream4 global interrupt.
 */
void DMA1_Stream4_IRQHandler (void) {
    HAL_DMA_IRQHandler (&DMAGetStreamById (eDMA_STREAM_4)->handle);
}

/**
 * @brief This function handles DMA1 stream5 global interrupt.
 */
void DMA1_Stream5_IRQHandler (void) {
    HAL_DMA_IRQHandler (&DMAGetStreamById (eDMA_STREAM_5)->handle);
}

/**
 * @brief This function handles DMA1 stream6 global interrupt.
 */
void DMA1_Stream6_IRQHandler (void) {
    HAL_DMA_IRQHandler (&DMAGetStreamById (eDMA_STREAM_6)->handle);
}

static eDMA_STREAM_ID_t DMAGetFreeStreamId (void) {

    for (uint32_t i = 0U; i < eDMA_STREAM_MAX; ++i) {
        if (gDMAStreams[i].isInUse == false) {
            return i;
        }
    }
    return eDMA_STREAM_MAX;
}

static DMA_Stream_TypeDef* DMAGetInstanceById (eDMA_STREAM_ID_t id) {

    switch (id) {
    case eDMA_STREAM_0: return DMA1_Stream0;
    case eDMA_STREAM_1: return DMA1_Stream1;
    case eDMA_STREAM_2: return DMA1_Stream2;
    case eDMA_STREAM_3: return DMA1_Stream3;
    case eDMA_STREAM_4: return DMA1_Stream4;
    case eDMA_STREAM_5: return DMA1_Stream5;
    case eDMA_STREAM_6: return DMA1_Stream6;
    default: return NULL;
    }
}

static eSTATUS_t DMAClockInit (DMAInitConf_t conf, DMAStream_t* pOutStream) {

    if (pOutStream == NULL) {
        LOG_ERROR ("dma config is NULL");
        return eSTATUS_FAILURE;
    }
    __HAL_RCC_DMA1_CLK_ENABLE ();
    return eSTATUS_SUCCESS;
}

eSTATUS_t DMAInit (DMAInitConf_t conf, eDMA_STREAM_ID_t* pOutStreamId) {

    if (pOutStreamId == NULL) {
        LOG_ERROR ("dma config is NULL");
        return eSTATUS_FAILURE;
    }

    if (conf.direction == DMA_MEMORY_TO_PERIPH && conf.fifoMode == DMA_FIFOMODE_ENABLE) {
        LOG_ERROR ("Direct mode (fifo disabled) has to be used for memory-to-peripheral transfers");
        return eSTATUS_FAILURE;
    }

    eDMA_STREAM_ID_t streamId = DMAGetFreeStreamId ();
    if (streamId == eDMA_STREAM_MAX) {
        LOG_ERROR ("No free DMA streams available");
        return eSTATUS_FAILURE;
    }

    DMAStream_t* pStream = DMAGetStreamById (streamId);
    if (pStream->isInitialized == true || pStream->isInUse == true) {
        LOG_ERROR ("DMA stream is already in use");
        return eSTATUS_FAILURE;
    }

    memset (pStream, 0, sizeof (DMAStream_t));
    pStream->streamId                        = streamId;
    pStream->handle.Instance                 = DMAGetInstanceById (streamId);
    pStream->handle.Init.Request             = conf.request;
    pStream->handle.Init.Priority            = conf.priority;
    pStream->handle.Init.Direction           = conf.direction;
    pStream->handle.Init.PeriphInc           = DMA_PINC_DISABLE;
    pStream->handle.Init.MemInc              = DMA_MINC_ENABLE;
    pStream->handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    pStream->handle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    pStream->handle.Init.Mode                = conf.transferMode;
    /*
     * When it is configured in direct mode ***(FIFO disabled)***, to
     * transfer data in memory-to-peripheral mode, the DMA preloads only
     * one data from the memory to the internal FIFO to ensure an immediate
     * data transfer as soon as a DMA request is triggered by a peripheral.
     */
    pStream->handle.Init.FIFOMode      = conf.fifoMode;
    pStream->handle.Init.FIFOThreshold = conf.fifoThreshold;
    pStream->handle.Init.MemBurst      = DMA_MBURST_SINGLE;
    pStream->handle.Init.PeriphBurst   = DMA_PBURST_SINGLE;

    if (DMAClockInit (conf, pStream) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize DMA clock");
        goto error;
    }

    if (HAL_DMA_Init (&pStream->handle) != HAL_OK) {
        LOG_ERROR ("Failed to initialize DMA");
        goto error;
    }

    if (DMAEnableInterrupts (streamId, 10) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to enable DMA interrupts for timer");
        goto error;
    }

    pStream->isInUse       = true;
    pStream->isInitialized = true;
    return eSTATUS_SUCCESS;

error:
    memset (pStream, 0, sizeof (DMAStream_t));
    return eSTATUS_FAILURE;
}

eSTATUS_t DMAEnableInterrupts (eDMA_STREAM_ID_t streamId, uint32_t priority) {

    DMAStream_t* pStream = DMAGetStreamById (streamId);
    if (pStream == NULL || pStream->isInitialized == false) {
        LOG_ERROR ("DMA stream is not initialized");
        return eSTATUS_FAILURE;
    }

    uint32_t interruptId = 0U;
    switch (streamId) {
    case eDMA_STREAM_0: interruptId = DMA1_Stream0_IRQn; break;
    case eDMA_STREAM_1: interruptId = DMA1_Stream1_IRQn; break;
    case eDMA_STREAM_2: interruptId = DMA1_Stream2_IRQn; break;
    case eDMA_STREAM_3: interruptId = DMA1_Stream3_IRQn; break;
    case eDMA_STREAM_4: interruptId = DMA1_Stream4_IRQn; break;
    case eDMA_STREAM_5: interruptId = DMA1_Stream5_IRQn; break;
    case eDMA_STREAM_6: interruptId = DMA1_Stream6_IRQn; break;
    default: LOG_ERROR ("Invalid DMA stream ID"); return eSTATUS_FAILURE;
    }

    HAL_NVIC_SetPriority (interruptId, priority, priority);
    HAL_NVIC_EnableIRQ (interruptId);

    return eSTATUS_SUCCESS;
}

DMAStream_t* DMAGetStreamById (eDMA_STREAM_ID_t id) {
    if (id >= eDMA_STREAM_MAX) {
        return NULL;
    }
    return &gDMAStreams[id];
}