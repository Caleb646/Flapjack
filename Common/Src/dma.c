#include "dma.h"
#include "common.h"
#include "hal.h"
#include "log/logger.h"
#include <stdint.h>

#define CHECK_DMA1_STREAM_VALID(stream)                                              \
    (                                                                                \
    (stream) == DMA1_Stream0 || (stream) == DMA1_Stream1 ||                          \
    (stream) == DMA1_Stream2 || (stream) == DMA1_Stream3 ||                          \
    (stream) == DMA1_Stream4 || (stream) == DMA1_Stream5 || (stream) == DMA1_Stream6 \
    )
#define CHECK_DMA_CONFIG_OK(pDMAConfig) \
    ((pDMAConfig) != NULL && (pDMAConfig)->pDMA != NULL && (pDMAConfig)->request != 0)

// DMA_HandleTypeDef gDMAStream_0                  = { 0 };
// DMA_HandleTypeDef gDMAStream_1                  = { 0 };
// DMA_HandleTypeDef gDMAStream_2                  = { 0 };
// DMA_HandleTypeDef gDMAStream_3                  = { 0 };
// DMA_HandleTypeDef gDMAStream_4                  = { 0 };
// DMA_HandleTypeDef gDMAStream_5                  = { 0 };
// DMA_HandleTypeDef gDMAStream_6                  = { 0 };
// uint8_t gDMAStreamsAlloced[7]                   = { 0 };
// DMACallback gDMAStreamCallbacks[7][4]           = { 0 };
// DMA_HandleTypeDef* gDMACallbackStreamHandles[7] = { 0 };

static DMAStream_t gDMAStreams[eDMA_STREAM_MAX] = { 0 };

static DMAStream_t* DMAGetStreamById (eDMA_STREAM_ID_t id);

/**
 * @brief This function handles DMA1 stream0 global interrupt.
 */
void DMA1_Stream0_IRQHandler (void) {
    HAL_DMA_IRQHandler (DMAGetStreamById (eDMA_STREAM_0));
}

/**
 * @brief This function handles DMA1 stream1 global interrupt.
 */
void DMA1_Stream1_IRQHandler (void) {
    HAL_DMA_IRQHandler (DMAGetStreamById (eDMA_STREAM_1));
}

/**
 * @brief This function handles DMA1 stream2 global interrupt.
 */
void DMA1_Stream2_IRQHandler (void) {
    HAL_DMA_IRQHandler (DMAGetStreamById (eDMA_STREAM_2));
}

/**
 * @brief This function handles DMA1 stream3 global interrupt.
 */
void DMA1_Stream3_IRQHandler (void) {
    HAL_DMA_IRQHandler (DMAGetStreamById (eDMA_STREAM_3));
}

/**
 * @brief This function handles DMA1 stream4 global interrupt.
 */
void DMA1_Stream4_IRQHandler (void) {
    HAL_DMA_IRQHandler (DMAGetStreamById (eDMA_STREAM_4));
}

/**
 * @brief This function handles DMA1 stream5 global interrupt.
 */
void DMA1_Stream5_IRQHandler (void) {
    HAL_DMA_IRQHandler (DMAGetStreamById (eDMA_STREAM_5));
}

/**
 * @brief This function handles DMA1 stream6 global interrupt.
 */
void DMA1_Stream6_IRQHandler (void) {
    HAL_DMA_IRQHandler (DMAGetStreamById (eDMA_STREAM_6));
}

static eDMA_STREAM_ID_t DMAGetFreeStreamId (void) {

    for (uint32_t i = 0U; i < eDMA_STREAM_MAX; ++i) {
        if (gDMAStreams[i].isInUse == FALSE) {
            return i;
        }
    }
    return eDMA_STREAM_MAX;
}

static DMA_TypeDef* DMAGetInstanceById (eDMA_STREAM_ID_t id) {

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

eSTATUS_t DMASystemInit (void) {
    LOG_INFO ("Initializing DMA1 system");
    // __HAL_RCC_DMA1_CLK_ENABLE ();

    // gDMAStream_0.Instance = DMA1_Stream0;
    // gDMAStream_1.Instance = DMA1_Stream1;
    // gDMAStream_2.Instance = DMA1_Stream2;
    // gDMAStream_3.Instance = DMA1_Stream3;
    // gDMAStream_4.Instance = DMA1_Stream4;
    // gDMAStream_5.Instance = DMA1_Stream5;
    // gDMAStream_6.Instance = DMA1_Stream6;

    // uint32_t p = 10;

    // HAL_NVIC_SetPriority (DMA1_Stream0_IRQn, p, p);
    // HAL_NVIC_EnableIRQ (DMA1_Stream0_IRQn);

    // HAL_NVIC_SetPriority (DMA1_Stream1_IRQn, p, p);
    // HAL_NVIC_EnableIRQ (DMA1_Stream1_IRQn);

    // HAL_NVIC_SetPriority (DMA1_Stream2_IRQn, p, p);
    // HAL_NVIC_EnableIRQ (DMA1_Stream2_IRQn);

    // HAL_NVIC_SetPriority (DMA1_Stream3_IRQn, p, p);
    // HAL_NVIC_EnableIRQ (DMA1_Stream3_IRQn);

    // HAL_NVIC_SetPriority (DMA1_Stream4_IRQn, p, p);
    // HAL_NVIC_EnableIRQ (DMA1_Stream4_IRQn);

    // HAL_NVIC_SetPriority (DMA1_Stream5_IRQn, p, p);
    // HAL_NVIC_EnableIRQ (DMA1_Stream5_IRQn);

    // HAL_NVIC_SetPriority (DMA1_Stream6_IRQn, p, p);
    // HAL_NVIC_EnableIRQ (DMA1_Stream6_IRQn);

    return eSTATUS_SUCCESS;
}

eSTATUS_t DMAClockInit (DMAInitConf_t const* pConf, DMAStream_t* pOutStream) {

    if (pConf == NULL || pOutStream == NULL) {
        LOG_ERROR ("dma config is NULL");
        return eSTATUS_FAILURE;
    }
    __HAL_RCC_DMA1_CLK_ENABLE ();
}

eSTATUS_t DMAInit (DMAInitConf_t const* pConf, eDMA_STREAM_ID_t* pOutStreamId) {

    if (pConf == NULL || pOutStreamId == NULL) {
        LOG_ERROR ("dma config is NULL");
        return eSTATUS_FAILURE;
    }

    if (pConf->direction == DMA_MEMORY_TO_PERIPH && pConf->fifoMode == DMA_FIFOMODE_ENABLE) {
        LOG_ERROR ("Direct mode (fifo disabled) has to be used for memory-to-peripheral transfers");
        return eSTATUS_FAILURE;
    }

    eDMA_STREAM_ID_t streamId = DMAGetFreeStreamId ();
    if (streamId == eDMA_STREAM_MAX) {
        LOG_ERROR ("No free DMA streams available");
        return eSTATUS_FAILURE;
    }

    DMAStream_t* pStream = DMAGetStreamById (streamId);
    if (pStream->isInitialized == TRUE || pStream->isInUse == TRUE) {
        LOG_ERROR ("DMA stream is already in use");
        return eSTATUS_FAILURE;
    }

    memset (pStream, 0, sizeof (DMAStream_t));
    pStream->streamId              = streamId;
    pStream->isInUse               = TRUE;
    pStream->isInitialized         = TRUE;
    pStream->handle.Instance       = DMAGetInstanceById (streamId);
    pStream->handle.Init.Request   = pConf->request;
    pStream->handle.Init.Priority  = pConf->priority;
    pStream->handle.Init.Direction = pConf->direction;
    pStream->handle.Init.PeriphInc = DMA_PINC_DISABLE;
    pStream->handle.Init.MemInc    = DMA_MINC_ENABLE;
    pStream->handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    pStream->handle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    pStream->handle.Init.Mode                = pConf->transferMode;
    /*
     * When it is configured in direct mode ***(FIFO disabled)***, to
     * transfer data in memory-to-peripheral mode, the DMA preloads only
     * one data from the memory to the internal FIFO to ensure an immediate
     * data transfer as soon as a DMA request is triggered by a peripheral.
     */
    pStream->handle.Init.FIFOMode      = pConf->fifoMode;
    pStream->handle.Init.FIFOThreshold = pConf->fifoThreshold;
    pStream->handle.Init.MemBurst      = DMA_MBURST_SINGLE;
    pStream->handle.Init.PeriphBurst   = DMA_PBURST_SINGLE;

    if (DMAClockInit (pConf, pStream) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize DMA clock");
        goto error;
    }

    if (HAL_DMA_Init (&pStream->handle) != HAL_OK) {
        LOG_ERROR ("Failed to initialize DMA");
        goto error;
    }
    return eSTATUS_SUCCESS;

error:
    memset (pStream, 0, sizeof (DMAStream_t));
    return eSTATUS_FAILURE;
}

eSTATUS_t DMAEnableInterrupts (eDMA_STREAM_ID_t streamId, uint32_t priority) {

    DMAStream_t* pStream = DMAGetStreamById (streamId);
    if (pStream == NULL || pStream->isInitialized == FALSE) {
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

eSTATUS_t DMASetTimCallback (DMA_HandleTypeDef* hdma, DMACallback cb, eDMA_CB_TYPE cbType) {

    if (hdma == NULL || cb == NULL) {
        LOG_ERROR ("Invalid DMA handle or callback");
        return eSTATUS_FAILURE;
    }

    int32_t idx = DMAStream2Idx (hdma->Instance);
    if (idx < 0) {
        LOG_ERROR ("Invalid DMA stream");
        return eSTATUS_FAILURE;
    }

    gDMACallbackStreamHandles[idx] = hdma;
    switch (cbType) {
    case eDMA_CB_TRANSFER_COMPLETE:
        gDMAStreamCallbacks[idx][eDMA_CB_TRANSFER_COMPLETE] = cb;
        break;
    case eDMA_CB_HALF_TRANSFER:
        gDMAStreamCallbacks[idx][eDMA_CB_HALF_TRANSFER] = cb;
        break;
    case eDMA_CB_TRANSFER_ERROR:
        gDMAStreamCallbacks[idx][eDMA_CB_TRANSFER_ERROR] = cb;
        break;
    case eDMA_CB_ABORT:
        gDMAStreamCallbacks[idx][eDMA_CB_ABORT] = cb;
        break;
    default:
        LOG_ERROR ("Invalid DMA callback type");
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

/*
 * If the all the streams are used this doesn't mean that the DMA is not
 * possible. Each stream supports multiple channels, so it can be shared.
 */
DMA_HandleTypeDef* DMAGetUnusedStreamHandle (void) {

    if (gDMAStreamsAlloced[0] == 0) {
        gDMAStreamsAlloced[0] = 1;
        return &gDMAStream_0;
    }

    if (gDMAStreamsAlloced[1] == 0) {
        gDMAStreamsAlloced[1] = 1;
        return &gDMAStream_1;
    }

    if (gDMAStreamsAlloced[2] == 0) {
        gDMAStreamsAlloced[2] = 1;
        return &gDMAStream_2;
    }

    if (gDMAStreamsAlloced[3] == 0) {
        gDMAStreamsAlloced[3] = 1;
        return &gDMAStream_3;
    }

    if (gDMAStreamsAlloced[4] == 0) {
        gDMAStreamsAlloced[4] = 1;
        return &gDMAStream_4;
    }

    if (gDMAStreamsAlloced[5] == 0) {
        gDMAStreamsAlloced[5] = 1;
        return &gDMAStream_5;
    }

    if (gDMAStreamsAlloced[6] == 0) {
        gDMAStreamsAlloced[6] = 1;
        return &gDMAStream_6;
    }
    LOG_ERROR ("No unused DMA streams available");
    return NULL;
}