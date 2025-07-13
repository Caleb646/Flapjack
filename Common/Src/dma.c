#include "dma.h"
#include "common.h"
#include "hal.h"
#include "log.h"
#include <stdint.h>

DMA_HandleTypeDef gDMAStream_0 = { 0 };
DMA_HandleTypeDef gDMAStream_1 = { 0 };
DMA_HandleTypeDef gDMAStream_2 = { 0 };
DMA_HandleTypeDef gDMAStream_3 = { 0 };
DMA_HandleTypeDef gDMAStream_4 = { 0 };
DMA_HandleTypeDef gDMAStream_5 = { 0 };
DMA_HandleTypeDef gDMAStream_6 = { 0 };

/**
 * @brief This function handles DMA1 stream0 global interrupt.
 */
void DMA1_Stream0_IRQHandler (void) {
    HAL_DMA_IRQHandler (&gDMAStream_0);
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


STATUS_TYPE DMASystemInit (void) {

    __HAL_RCC_DMA1_CLK_ENABLE ();

    HAL_NVIC_SetPriority (DMA1_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ (DMA1_Stream0_IRQn);

    HAL_NVIC_SetPriority (DMA1_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ (DMA1_Stream1_IRQn);

    HAL_NVIC_SetPriority (DMA1_Stream2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ (DMA1_Stream2_IRQn);

    HAL_NVIC_SetPriority (DMA1_Stream3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ (DMA1_Stream3_IRQn);

    HAL_NVIC_SetPriority (DMA1_Stream4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ (DMA1_Stream4_IRQn);

    HAL_NVIC_SetPriority (DMA1_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ (DMA1_Stream5_IRQn);

    HAL_NVIC_SetPriority (DMA1_Stream6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ (DMA1_Stream6_IRQn);

    return eSTATUS_SUCCESS;
}

STATUS_TYPE DMAInit (DMAConfig* pConfig, DMA_HandleTypeDef** ppOutHandle) {
    if (pConfig == NULL || pConfig->pDMA == NULL || ppOutHandle == NULL) {
        LOG_ERROR ("DMA configuration or output handle is NULL");
        return eSTATUS_FAILURE;
    }

    DMA_HandleTypeDef hdma        = { 0 };
    hdma.Instance                 = pConfig->pDMA;
    hdma.Init.Direction           = pConfig->direction;
    hdma.Init.Priority            = pConfig->priority;
    hdma.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma.Init.MemInc              = DMA_MINC_ENABLE;
    hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma.Init.Mode                = DMA_NORMAL;
    hdma.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    hdma.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_1QUARTERFULL;
    hdma.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma.Init.PeriphBurst         = DMA_PBURST_SINGLE;

    if (HAL_DMA_Init (&hdma) != HAL_OK) {
        LOG_ERROR ("Failed to initialize DMA");
        return eSTATUS_FAILURE;
    }

    if (pConfig->pDMA == DMA1_Stream0) {
        gDMAStream_0 = hdma;
        *ppOutHandle = &gDMAStream_0;
    } else if (pConfig->pDMA == DMA1_Stream1) {
        gDMAStream_1 = hdma;
        *ppOutHandle = &gDMAStream_1;
    } else if (pConfig->pDMA == DMA1_Stream2) {
        gDMAStream_2 = hdma;
        *ppOutHandle = &gDMAStream_2;
    } else if (pConfig->pDMA == DMA1_Stream3) {
        gDMAStream_3 = hdma;
        *ppOutHandle = &gDMAStream_3;
    } else if (pConfig->pDMA == DMA1_Stream4) {
        gDMAStream_4 = hdma;
        *ppOutHandle = &gDMAStream_4;
    } else if (pConfig->pDMA == DMA1_Stream5) {
        gDMAStream_5 = hdma;
        *ppOutHandle = &gDMAStream_5;
    } else if (pConfig->pDMA == DMA1_Stream6) {
        gDMAStream_6 = hdma;
        *ppOutHandle = &gDMAStream_6;
    } else {
        LOG_ERROR ("Unsupported DMA stream");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

/*
 * If the all the streams are used this doesn't mean that the DMA is not
 * possible. Each stream supports multiple channels, so it can be shared.
 */
DMA_HandleTypeDef* DMAGetUnusedStreamHandle (void) {

    if (gDMAStream_0.Instance == NULL) {
        return &gDMAStream_0;
    }

    if (gDMAStream_1.Instance == NULL) {
        return &gDMAStream_1;
    }

    if (gDMAStream_2.Instance == NULL) {
        return &gDMAStream_2;
    }

    if (gDMAStream_3.Instance == NULL) {
        return &gDMAStream_3;
    }

    if (gDMAStream_4.Instance == NULL) {
        return &gDMAStream_4;
    }

    if (gDMAStream_5.Instance == NULL) {
        return &gDMAStream_5;
    }

    if (gDMAStream_6.Instance == NULL) {
        return &gDMAStream_6;
    }
    LOG_ERROR ("No unused DMA streams available");
    return NULL;
}