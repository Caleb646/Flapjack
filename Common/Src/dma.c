#include "dma.h"
#include "common.h"
#include "hal.h"
#include "log.h"
#include <stdint.h>

#define CHECK_DMA1_STREAM_VALID(stream)                     \
    (                                                       \
    (stream) == DMA1_Stream0 || (stream) == DMA1_Stream1 || \
    (stream) == DMA1_Stream2 || (stream) == DMA1_Stream3 || \
    (stream) == DMA1_Stream4 || (stream) == DMA1_Stream5 || (stream) == DMA1_Stream6)
#define CHECK_DMA_CONFIG_OK(pDMAConfig) \
    ((pDMAConfig) != NULL && (pDMAConfig)->pDMA != NULL && (pDMAConfig)->request != 0)

DMA_HandleTypeDef gDMAStream_0                  = { 0 };
DMA_HandleTypeDef gDMAStream_1                  = { 0 };
DMA_HandleTypeDef gDMAStream_2                  = { 0 };
DMA_HandleTypeDef gDMAStream_3                  = { 0 };
DMA_HandleTypeDef gDMAStream_4                  = { 0 };
DMA_HandleTypeDef gDMAStream_5                  = { 0 };
DMA_HandleTypeDef gDMAStream_6                  = { 0 };
uint8_t gDMAStreamsAlloced[7]                   = { 0 };
DMACallback gDMAStreamCallbacks[7][4]           = { 0 };
DMA_HandleTypeDef* gDMACallbackStreamHandles[7] = { 0 };

/**
 * @brief This function handles DMA1 stream0 global interrupt.
 */
void DMA1_Stream0_IRQHandler (void) {
    // LOG_INFO ("DMA1 Stream0 IRQ Handler");
    HAL_DMA_IRQHandler (&gDMAStream_0);
    // LOG_INFO ("irq");
}

/**
 * @brief This function handles DMA1 stream1 global interrupt.
 */
void DMA1_Stream1_IRQHandler (void) {
    HAL_DMA_IRQHandler (&gDMAStream_1);
}

/**
 * @brief This function handles DMA1 stream2 global interrupt.
 */
void DMA1_Stream2_IRQHandler (void) {
    HAL_DMA_IRQHandler (&gDMAStream_2);
}

/**
 * @brief This function handles DMA1 stream3 global interrupt.
 */
void DMA1_Stream3_IRQHandler (void) {
    HAL_DMA_IRQHandler (&gDMAStream_3);
}

/**
 * @brief This function handles DMA1 stream4 global interrupt.
 */
void DMA1_Stream4_IRQHandler (void) {
    HAL_DMA_IRQHandler (&gDMAStream_4);
}

/**
 * @brief This function handles DMA1 stream5 global interrupt.
 */
void DMA1_Stream5_IRQHandler (void) {
    HAL_DMA_IRQHandler (&gDMAStream_5);
}

/**
 * @brief This function handles DMA1 stream6 global interrupt.
 */
void DMA1_Stream6_IRQHandler (void) {
    HAL_DMA_IRQHandler (&gDMAStream_6);
}


static int32_t DMAStream2Idx (DMA_TypeDef* pStream) {
    if (pStream == DMA1_Stream0) {
        return 0;
    }
    if (pStream == DMA1_Stream1) {
        return 1;
    }
    if (pStream == DMA1_Stream2) {
        return 2;
    }
    if (pStream == DMA1_Stream3) {
        return 3;
    }
    if (pStream == DMA1_Stream4) {
        return 4;
    }
    if (pStream == DMA1_Stream5) {
        return 5;
    }
    if (pStream == DMA1_Stream6) {
        return 6;
    }
    return -1;
}

static DMA_HandleTypeDef* DMAGetHandleByStream (DMA_TypeDef* pStream) {

    if (pStream == DMA1_Stream0) {
        return &gDMAStream_0;
    } else if (pStream == DMA1_Stream1) {
        return &gDMAStream_1;
    } else if (pStream == DMA1_Stream2) {
        return &gDMAStream_2;
    } else if (pStream == DMA1_Stream3) {
        return &gDMAStream_3;
    } else if (pStream == DMA1_Stream4) {
        return &gDMAStream_4;
    } else if (pStream == DMA1_Stream5) {
        return &gDMAStream_5;
    } else if (pStream == DMA1_Stream6) {
        return &gDMAStream_6;
    }
    return NULL;
}

STATUS_TYPE DMASystemInit (void) {
    LOG_INFO ("Initializing DMA1 system");
    __HAL_RCC_DMA1_CLK_ENABLE ();

    gDMAStream_0.Instance = DMA1_Stream0;
    gDMAStream_1.Instance = DMA1_Stream1;
    gDMAStream_2.Instance = DMA1_Stream2;
    gDMAStream_3.Instance = DMA1_Stream3;
    gDMAStream_4.Instance = DMA1_Stream4;
    gDMAStream_5.Instance = DMA1_Stream5;
    gDMAStream_6.Instance = DMA1_Stream6;

    uint32_t p = 6;

    HAL_NVIC_SetPriority (DMA1_Stream0_IRQn, p, p);
    HAL_NVIC_EnableIRQ (DMA1_Stream0_IRQn);

    HAL_NVIC_SetPriority (DMA1_Stream1_IRQn, p, p);
    HAL_NVIC_EnableIRQ (DMA1_Stream1_IRQn);

    HAL_NVIC_SetPriority (DMA1_Stream2_IRQn, p, p);
    HAL_NVIC_EnableIRQ (DMA1_Stream2_IRQn);

    HAL_NVIC_SetPriority (DMA1_Stream3_IRQn, p, p);
    HAL_NVIC_EnableIRQ (DMA1_Stream3_IRQn);

    HAL_NVIC_SetPriority (DMA1_Stream4_IRQn, p, p);
    HAL_NVIC_EnableIRQ (DMA1_Stream4_IRQn);

    HAL_NVIC_SetPriority (DMA1_Stream5_IRQn, p, p);
    HAL_NVIC_EnableIRQ (DMA1_Stream5_IRQn);

    HAL_NVIC_SetPriority (DMA1_Stream6_IRQn, p, p);
    HAL_NVIC_EnableIRQ (DMA1_Stream6_IRQn);

    return eSTATUS_SUCCESS;
}

STATUS_TYPE DMAInit (DMAConfig config, DMA_HandleTypeDef** ppOutHandle) {
    if (ppOutHandle == NULL) {
        LOG_ERROR ("Output handle is NULL");
        return eSTATUS_FAILURE;
    }

    if (CHECK_DMA_CONFIG_OK (&config) == FALSE) {
        LOG_ERROR ("Invalid DMA configuration");
        return eSTATUS_FAILURE;
    }

    if (CHECK_DMA1_STREAM_VALID (config.pDMA) == FALSE) {
        LOG_ERROR ("Invalid DMA stream");
        return eSTATUS_FAILURE;
    }

    DMA_HandleTypeDef hdma        = { 0 };
    hdma.Instance                 = config.pDMA;
    hdma.Init.Request             = config.request;
    hdma.Init.Direction           = config.direction;
    hdma.Init.Priority            = config.priority;
    hdma.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma.Init.MemInc              = DMA_MINC_ENABLE;
    hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma.Init.Mode                = DMA_NORMAL;
    /*
     * When it is configured in direct mode ***(FIFO disabled)***, to
     * transfer data in memory-to-peripheral mode, the DMA preloads only
     * one data from the memory to the internal FIFO to ensure an immediate
     * data transfer as soon as a DMA request is triggered by a peripheral.
     */
    hdma.Init.FIFOMode      = DMA_FIFOMODE_DISABLE;
    hdma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma.Init.MemBurst      = DMA_MBURST_SINGLE;
    hdma.Init.PeriphBurst   = DMA_PBURST_SINGLE;

    if (hdma.Init.Direction == eDMA_DIRECTION_MEMORY_TO_PERIPH && hdma.Init.FIFOMode == DMA_FIFOMODE_ENABLE) {
        LOG_ERROR ("direct mode (fifo disabled) has to be used for memory-to-peripheral transfers");
        return eSTATUS_FAILURE;
    }

    DMA_HandleTypeDef* pHandle = DMAGetHandleByStream (hdma.Instance);
    if (pHandle == NULL) {
        LOG_ERROR ("Failed to get DMA handle for stream");
        return eSTATUS_FAILURE;
    }
    *pHandle     = hdma;
    *ppOutHandle = pHandle;

    if (HAL_DMA_Init (*ppOutHandle) != HAL_OK) {
        LOG_ERROR ("Failed to initialize DMA");
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

STATUS_TYPE DMASetTimCallback (DMA_HandleTypeDef* hdma, DMACallback cb, eDMA_CB_TYPE cbType) {

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