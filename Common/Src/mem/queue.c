#include "mem/queue.h"
#include <string.h>


STATUS_TYPE QueueInit (Queue* pQueue, void* pBuffer, uint16_t capacity, uint16_t elementSize) {
    if (pQueue == NULL || pBuffer == NULL || capacity == 0 || elementSize == 0) {
        return eSTATUS_FAILURE;
    }
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

uint8_t QueueIsEmpty (Queue const* pQueue) {
    if (pQueue == NULL) {
        return 1; // Consider NULL queue as empty
    }
    return pQueue->count == 0;
}

uint8_t QueueIsFull (Queue const* pQueue) {
    if (pQueue == NULL) {
        return 1; // Consider NULL queue as full
    }
    return pQueue->count == pQueue->capacity;
}

uint16_t QueueCount (Queue const* pQueue) {
    if (pQueue == NULL) {
        return 0;
    }
    return pQueue->count;
}

uint16_t QueueCapacity (Queue const* pQueue) {
    if (pQueue == NULL) {
        return 0;
    }
    return pQueue->capacity;
}

STATUS_TYPE QueueEnqueue (Queue* pQueue, void const* pElement) {
    if (pQueue == NULL || pElement == NULL) {
        return eSTATUS_FAILURE;
    }

    if (QueueIsFull (pQueue)) {
        return eSTATUS_FAILURE;
    }

    // Calculate the memory location for the tail element
    uint8_t* pDataBytes = (uint8_t*)pQueue->pData;
    uint8_t* pTailLocation = pDataBytes + (size_t)(pQueue->tail * pQueue->elementSize);

    // Copy the element to the tail position
    memcpy (pTailLocation, pElement, pQueue->elementSize);

    // Update tail index (circular)
    pQueue->tail = (pQueue->tail + 1U) &
                   (pQueue->capacity - 1U); // Use bitwise AND for circular indexing
    pQueue->count++;

    return eSTATUS_SUCCESS;
}

STATUS_TYPE QueueDequeue (Queue* pQueue, void* pOutElement) {
    if (pQueue == NULL || pOutElement == NULL) {
        return eSTATUS_FAILURE;
    }

    if (QueueIsEmpty (pQueue)) {
        return eSTATUS_FAILURE;
    }

    // Calculate the memory location for the head element
    uint8_t* pDataBytes = (uint8_t*)pQueue->pData;
    uint8_t* pHeadLocation = pDataBytes + (size_t)(pQueue->head * pQueue->elementSize);

    // Copy the element from the head position
    memcpy (pOutElement, pHeadLocation, pQueue->elementSize);

    // Update head index (circular)
    pQueue->head = (pQueue->head + 1U) &
                   (pQueue->capacity - 1U); // Use bitwise AND for circular indexing
    pQueue->count--;

    return eSTATUS_SUCCESS;
}

STATUS_TYPE QueuePeek (Queue const* pQueue, void* pOutElement) {
    if (pQueue == NULL || pOutElement == NULL) {
        return eSTATUS_FAILURE;
    }

    if (QueueIsEmpty (pQueue)) {
        return eSTATUS_FAILURE;
    }

    // Calculate the memory location for the head element
    uint8_t const* pDataBytes = (uint8_t const*)pQueue->pData;
    uint8_t const* pHeadLocation =
    pDataBytes + (size_t)(pQueue->head * pQueue->elementSize);

    // Copy the element from the head position without removing it
    memcpy (pOutElement, pHeadLocation, pQueue->elementSize);

    return eSTATUS_SUCCESS;
}

void QueueClear (Queue* pQueue) {
    if (pQueue == NULL) {
        return;
    }

    pQueue->head  = 0;
    pQueue->tail  = 0;
    pQueue->count = 0;
}
