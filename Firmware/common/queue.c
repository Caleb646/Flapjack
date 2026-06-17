#include "common/queue.h"
#include "core/core.h"
#include "hal.h"
#include "common/mem.h"
#include <stdint.h>
#include <string.h>

// #define SEMAPHORE_ID            1U
#define IS_QUEUE_SHARED(pQueue) ((pQueue)->processID != 0U)

eSTATUS_t QueueInit (Queue_t* pQueue, void* pBuffer, uint16_t capacity, uint16_t elementSize, bool isShared) {

    FJ_UNUSED (isShared);

    if (pQueue == NULL || pBuffer == NULL || capacity == 0 || elementSize == 0) {
        return eSTATUS_FAILURE;
    }
    memset (pQueue, 0, sizeof (Queue_t));
    memset (pBuffer, 0, (uint32_t)capacity * (uint32_t)elementSize);
    // Check if capacity is a power of 2
    if ((capacity & (capacity - 1U)) != 0U) {
        // Not a power of 2
        return eSTATUS_FAILURE;
    }

    pQueue->pData       = pBuffer;
    pQueue->capacity    = capacity;
    pQueue->elementSize = elementSize;
    pQueue->head        = 0;
    pQueue->tail        = 0;
    pQueue->count       = 0;
    return eSTATUS_SUCCESS;
}

// eSTATUS_t
// QueueInit_SharedMemory (void* pMemoryStart, uint32_t memorySize, uint16_t capacity, uint16_t elementSize) {

//     /* Add 3 for potential 4 byte alignment */
//     uint32_t totalSize = sizeof (Queue_t) + 3U + (uint32_t)capacity *
//     (uint32_t)elementSize; if (totalSize > memorySize) {
//         return eSTATUS_FAILURE;
//     }

//     void* pQueueBuffer =
//     (void*)MEM_U32_ALIGN4 ((uint32_t)pMemoryStart + sizeof (Queue_t));
//     if (QueueInit ((Queue_t*)pMemoryStart, pQueueBuffer, capacity, elementSize, true) !=
//         eSTATUS_SUCCESS) {
//         return eSTATUS_FAILURE;
//     }
//     return eSTATUS_SUCCESS;
// }

bool QueueIsEmpty (Queue_t const* pQueue) {
    if (pQueue == NULL) {
        return true; // Consider NULL queue as empty
    }
    return pQueue->count == 0;
}

bool QueueIsFull (Queue_t const* pQueue) {
    if (pQueue == NULL) {
        return true; // Consider NULL queue as full
    }
    return pQueue->count == pQueue->capacity;
}

uint16_t QueueGetElementCount (Queue_t const* pQueue) {
    if (pQueue == NULL) {
        return 0;
    }
    return pQueue->count;
}

uint16_t QueueGetCapacity (Queue_t const* pQueue) {
    if (pQueue == NULL) {
        return 0;
    }
    return pQueue->capacity;
}

eSTATUS_t Queue_Push (Queue_t* pQueue, void const* pElement) {
    if (pQueue == NULL || pElement == NULL) {
        return eSTATUS_FAILURE;
    }

    if (QueueIsFull (pQueue)) {
        return eSTATUS_FAILURE;
    }

    // Calculate the memory location for the tail element
    uint8_t* pDataBytes    = (uint8_t*)pQueue->pData;
    uint8_t* pTailLocation = pDataBytes + (uint32_t)(pQueue->tail * pQueue->elementSize);

    // Copy the element to the tail position
    memcpy (pTailLocation, pElement, pQueue->elementSize);

    // Update tail index (circular)
    pQueue->tail = (pQueue->tail + 1U) & (pQueue->capacity - 1U); // Use bitwise AND for circular indexing
    pQueue->count++;
    return eSTATUS_SUCCESS;
}

eSTATUS_t Queue_Pop (Queue_t* pQueue, void* pOutElement) {
    if (pQueue == NULL || pOutElement == NULL) {
        return eSTATUS_FAILURE;
    }

    if (QueueIsEmpty (pQueue)) {
        return eSTATUS_FAILURE;
    }

    // Calculate the memory location for the head element
    uint8_t* pDataBytes    = (uint8_t*)pQueue->pData;
    uint8_t* pHeadLocation = pDataBytes + (uint32_t)(pQueue->head * pQueue->elementSize);

    // Copy the element from the head position
    memcpy (pOutElement, pHeadLocation, pQueue->elementSize);

    pQueue->count--;
    // Update head index (circular)
    pQueue->head = (pQueue->head + 1U) & (pQueue->capacity - 1U); // Use bitwise AND for circular indexing
    return eSTATUS_SUCCESS;
}

eSTATUS_t Queue_Peek (Queue_t const* pQueue, void* pOutElement) {
    if (pQueue == NULL || pOutElement == NULL) {
        return eSTATUS_FAILURE;
    }

    if (QueueIsEmpty (pQueue)) {
        return eSTATUS_FAILURE;
    }

    // Calculate the memory location for the head element
    uint8_t const* pDataBytes    = (uint8_t const*)pQueue->pData;
    uint8_t const* pHeadLocation = pDataBytes + (uint32_t)(pQueue->head * pQueue->elementSize);

    // Copy the element from the head position without removing it
    memcpy (pOutElement, pHeadLocation, pQueue->elementSize);

    return eSTATUS_SUCCESS;
}

void Queue_Clear (Queue_t* pQueue) {
    if (pQueue == NULL) {
        return;
    }

    pQueue->head  = 0;
    pQueue->tail  = 0;
    pQueue->count = 0;
}
