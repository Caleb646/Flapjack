#ifndef DMA_H
#define DMA_H
#include "common.h"
#include "hal.h"
#include "log.h"
#include <stdint.h>

#define DMA_CHECK_CONF_OK(pDMAConf) \
    ((pDMAConf) != NULL && (pDMAConf)->pDMA != NULL)

typedef enum {
    eDMA_DIRECTION_PERIPH_TO_MEMORY = DMA_PERIPH_TO_MEMORY,
    eDMA_DIRECTION_MEMORY_TO_PERIPH = DMA_MEMORY_TO_PERIPH,
    eDMA_DIRECTION_MEMORY_TO_MEMORY = DMA_MEMORY_TO_MEMORY
} eDMA_TRANSFER_DIR;

typedef enum {
    eDMA_PRIORITY_LOW       = DMA_PRIORITY_LOW,
    eDMA_PRIORITY_MEDIUM    = DMA_PRIORITY_MEDIUM,
    eDMA_PRIORITY_HIGH      = DMA_PRIORITY_HIGH,
    eDMA_PRIORITY_VERY_HIGH = DMA_PRIORITY_VERY_HIGH
} eDMA_PRIORITY;

typedef struct {
    DMA_Stream_TypeDef* pDMA;
    eDMA_TRANSFER_DIR direction;
    eDMA_PRIORITY priority;
    uint32_t request; // DMA request type, e.g., DMA_REQUEST_USART1_TX
} DMAConfig;

// Forward declaration of the PWM DMA handle structure
// typedef struct __PWMDMAHandle PWM_DMAHandle;

STATUS_TYPE DMASystemInit (void);
STATUS_TYPE DMAInit (DMAConfig config, DMA_HandleTypeDef** ppOutHandle);
DMA_HandleTypeDef* DMAGetUnusedStreamHandle (void);

#endif /* DMA_H */