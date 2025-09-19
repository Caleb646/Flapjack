#ifndef DMA_H
#define DMA_H
#include "common.h"
#include "hal.h"
#include "log/logger.h"
#include <stdint.h>

#define DMA_CHECK_CONF_OK(pDMAConf) \
    ((pDMAConf) != NULL && (pDMAConf)->pDMA != NULL)

#define DMA_CREATE_PWM_CONF(DMA_STREAM, DIRECTION, PRIORITY, DMA_REQUEST_ID) \
    { .pDMA          = (DMA_STREAM),                                         \
      .direction     = (DIRECTION),                                          \
      .priority      = (PRIORITY),                                           \
      .request       = (DMA_REQUEST_ID),                                     \
      .transferMode  = DMA_NORMAL,                                           \
      .fifoMode      = DMA_FIFOMODE_DISABLE, /* Direct Mode */               \
      .fifoThreshold = DMA_FIFO_THRESHOLD_FULL }

typedef uint8_t eDMA_STREAM_ID_t;
enum {
    eDMA_STREAM_0 = 0,
    eDMA_STREAM_1,
    eDMA_STREAM_2,
    eDMA_STREAM_3,
    eDMA_STREAM_4,
    eDMA_STREAM_5,
    eDMA_STREAM_6,
    eDMA_STREAM_MAX
};

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
    eDMA_TRANSFER_DIR direction;
    eDMA_PRIORITY priority;
    uint32_t request;      // DMA request type, e.g., DMA_REQUEST_USART1_TX
    uint32_t transferMode; // DMA transfer mode, e.g., DMA_NORMAL, DMA_CIRCULAR
    uint32_t fifoMode; // DMA FIFO mode, e.g., DMA_FIFOMODE_DISABLE (Direct Mode), DMA_FIFOMODE_ENABLE
    uint32_t fifoThreshold; // FIFO threshold level
} DMAInitConf_t;

typedef uint8_t eDMA_CALLBACK_t;
enum {
    eDMA_CALLBACK_TRANSFER_COMPLETE = 0,
    eDMA_CALLBACK_HALF_TRANSFER,
    eDMA_CALLBACK_TRANSFER_ERROR,
    eDMA_CALLBACK_ABORT
};

typedef void (*DMACallback_t) (struct __DMA_HandleTypeDef* hdma);

typedef struct {
    DMA_HandleTypeDef handle;
    eDMA_STREAM_ID_t streamId;
    DMACallback_t transferCompleteCallback;
    DMACallback_t halfTransferCallback;
    DMACallback_t transferErrorCallback;
    DMACallback_t abortCallback;
    BOOL_t isInUse;
    BOOL_t isInitialized;
} DMAStream_t;

// Forward declaration of the PWM DMA handle structure
// typedef struct __PWMDMAHandle PWM_DMAHandle;

eSTATUS_t DMASystemInit (void);
eSTATUS_t DMAInit (DMAConfig config, DMA_HandleTypeDef** ppOutHandle);
DMAStream_t* DMAGetStreamById (eDMA_STREAM_ID_t id);
DMA_HandleTypeDef* DMAGetUnusedStreamHandle (void);

#endif /* DMA_H */