#include "dma.h"
#include "common.h"
#include "hal.h"
#include "log.h"
#include <stdint.h>

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
    // HAL_DMA_IRQHandler (&hdma_tim2_ch3_up);
}

/**
 * @brief This function handles DMA1 stream1 global interrupt.
 */
void DMA1_Stream1_IRQHandler (void) {
    // HAL_DMA_IRQHandler (&hdma_tim2_ch3_up);
}

/**
 * @brief This function handles DMA1 stream2 global interrupt.
 */
void DMA1_Stream2_IRQHandler (void) {
    // HAL_DMA_IRQHandler (&hdma_tim2_ch3_up);
}

/**
 * @brief This function handles DMA1 stream3 global interrupt.
 */
void DMA1_Stream3_IRQHandler (void) {
    // HAL_DMA_IRQHandler (&hdma_tim5_ch4_trig);
}

/**
 * @brief This function handles DMA1 stream4 global interrupt.
 */
void DMA1_Stream4_IRQHandler (void) {
    // HAL_DMA_IRQHandler (&hdma_tim5_ch2);
}

/**
 * @brief This function handles DMA1 stream5 global interrupt.
 */
void DMA1_Stream5_IRQHandler (void) {
    // HAL_DMA_IRQHandler (&hdma_tim2_ch1);
}

/**
 * @brief This function handles DMA1 stream6 global interrupt.
 */
void DMA1_Stream6_IRQHandler (void) {
    // HAL_DMA_IRQHandler (&hdma_tim2_ch1);
}


STATUS_TYPE DMASystemInit (void) {

    __HAL_RCC_DMA1_CLK_ENABLE ();

    HAL_NVIC_SetPriority (DMA1_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ (DMA1_Stream1_IRQn);

    HAL_NVIC_SetPriority (DMA1_Stream3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ (DMA1_Stream3_IRQn);

    HAL_NVIC_SetPriority (DMA1_Stream4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ (DMA1_Stream4_IRQn);

    HAL_NVIC_SetPriority (DMA1_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ (DMA1_Stream5_IRQn);

    return eSTATUS_SUCCESS;
}

STATUS_TYPE DMAInit (DMA_HandleTypeDef* pHandle) {
    if (pHandle == NULL) {
        LOG_ERROR ("DMA handle is NULL");
        return eSTATUS_FAILURE;
    }

    // Initialize the DMA handle
    HAL_StatusTypeDef halStatus = HAL_DMA_Init (pHandle);
    if (halStatus != HAL_OK) {
        LOG_ERROR ("Failed to initialize DMA: %d", halStatus);
        return eSTATUS_FAILURE;
    }

    LOG_INFO ("DMA initialized successfully");
    return eSTATUS_SUCCESS;
}