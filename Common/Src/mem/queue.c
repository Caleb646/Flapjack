#include "mem/queue.h"
#include "common.h"
#include "hal.h"
#include "mem/mem.h"
#include <stdint.h>
#include <string.h>

#define SEMAPHORE_ID            1U
#define IS_QUEUE_SHARED(pQueue) ((pQueue)->processID != 0U)

/*
 * Start at 1 because 0 is reserved for NON shared queues
 */
static uint32_t g_ProcessID = 1;

#ifndef UNIT_TEST

static eSTATUS_t Queue_EnterCritical (Queue const* pQueue);
static void Queue_ExitCritical (Queue const* pQueue);

#endif // UNIT_TEST

static eSTATUS_t Queue_EnterCritical (Queue const* pQueue) {
    /*
     * Disable interrupts before taking the semaphore.
     * Do not want to enter an interrupt after taking the semaphore.
     * That could result in a deadlock if the interrupt tries to take the same semaphore.
     */
    // TODO: should save the current interrupt state
    // uint32_t primask = __get_PRIMASK ();

#if 0
    // __disable_irq ();
    portDISABLE_INTERRUPTS ();
    /*
     * Ensure the disable irq does not get reordered by CPU or compiler
     */
    // __DMB ();
    // clang-format off
    int32_t timeout = 10000;
    while(HAL_HSEM_Take (SEMAPHORE_ID, pQueue->processID) != HAL_OK && timeout-- > 0) {}
    // clang-format on
    if (timeout <= 0) {
        // __enable_irq ();
        portENABLE_INTERRUPTS ();
        return eSTATUS_FAILURE;
    }
    /*
     * Ensure enter critical stores/loads are completed
     */
    // __DMB ();
#endif
    return eSTATUS_SUCCESS;
}

static void Queue_ExitCritical (Queue const* pQueue) {
#if 0
    HAL_HSEM_Release (SEMAPHORE_ID, pQueue->processID);
    portENABLE_INTERRUPTS ();
    // __enable_irq ();
#endif
}

eSTATUS_t
QueueInit (Queue* pQueue, void* pBuffer, uint16_t capacity, uint16_t elementSize, BOOL_t isShared) {
    if (pQueue == NULL || pBuffer == NULL || capacity == 0 || elementSize == 0) {
        return eSTATUS_FAILURE;
    }
    memset (pQueue, 0, sizeof (Queue));
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

    if (isShared == TRUE) {
        if ((g_ProcessID + 1U) > 256U) {
            return eSTATUS_FAILURE;
        }
        pQueue->processID = g_ProcessID++;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t
QueueInit_SharedMemory (void* pMemoryStart, uint32_t memorySize, uint16_t capacity, uint16_t elementSize) {

    /* Add 3 for potential 4 byte alignment */
    uint32_t totalSize = sizeof (Queue) + 3U + (uint32_t)capacity * (uint32_t)elementSize;
    if (totalSize > memorySize) {
        return eSTATUS_FAILURE;
    }

    void* pQueueBuffer =
    (void*)MEM_U32_ALIGN4 ((uint32_t)pMemoryStart + sizeof (Queue));
    if (QueueInit ((Queue*)pMemoryStart, pQueueBuffer, capacity, elementSize, TRUE) != eSTATUS_SUCCESS) {
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

BOOL_t QueueIsEmpty (Queue const* pQueue) {
    if (pQueue == NULL) {
        return TRUE; // Consider NULL queue as empty
    }
    return pQueue->count == 0;
}

BOOL_t QueueIsFull (Queue const* pQueue) {
    if (pQueue == NULL) {
        return TRUE; // Consider NULL queue as full
    }
    return pQueue->count == pQueue->capacity;
}

uint16_t QueueGetElementCount (Queue const* pQueue) {
    if (pQueue == NULL) {
        return 0;
    }
    return pQueue->count;
}

uint16_t QueueGetCapacity (Queue const* pQueue) {
    if (pQueue == NULL) {
        return 0;
    }
    return pQueue->capacity;
}

eSTATUS_t Queue_Push (Queue* pQueue, void const* pElement) {
    if (pQueue == NULL || pElement == NULL) {
        return eSTATUS_FAILURE;
    }

    if (QueueIsFull (pQueue)) {
        return eSTATUS_FAILURE;
    }

    // Calculate the memory location for the tail element
    uint8_t* pDataBytes = (uint8_t*)pQueue->pData;
    uint8_t* pTailLocation =
    pDataBytes + (uint32_t)(pQueue->tail * pQueue->elementSize);

    // Copy the element to the tail position
    memcpy (pTailLocation, pElement, pQueue->elementSize);

    // Update tail index (circular)
    pQueue->tail = (pQueue->tail + 1U) &
                   (pQueue->capacity - 1U); // Use bitwise AND for circular indexing

    /*
     * NOTE: The reason for only protecting the count update is because
     * right now the only way the queue is used is 1 core pushes and 1 core
     * pops or 1 interrupt pushes and 1 core pops. There is not a case
     * where 1 core pushes & pops while another core pushes & pops
     */
    if (IS_QUEUE_SHARED (pQueue) == TRUE) {
        if (Queue_EnterCritical (pQueue) != eSTATUS_SUCCESS) {
            return eSTATUS_FAILURE;
        }
        pQueue->count++;
        Queue_ExitCritical (pQueue);
    } else {
        pQueue->count++;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t Queue_Pop (Queue* pQueue, void* pOutElement) {
    if (pQueue == NULL || pOutElement == NULL) {
        return eSTATUS_FAILURE;
    }

    if (QueueIsEmpty (pQueue)) {
        return eSTATUS_FAILURE;
    }

    // Calculate the memory location for the head element
    uint8_t* pDataBytes = (uint8_t*)pQueue->pData;
    uint8_t* pHeadLocation =
    pDataBytes + (uint32_t)(pQueue->head * pQueue->elementSize);

    // Copy the element from the head position
    memcpy (pOutElement, pHeadLocation, pQueue->elementSize);

    // Update head index (circular)
    pQueue->head = (pQueue->head + 1U) &
                   (pQueue->capacity - 1U); // Use bitwise AND for circular indexing

    /*
     * NOTE: The reason for only protecting the count update is because
     * right now the only way the queue is used is 1 core pushes and 1 core
     * pops or 1 interrupt pushes and 1 core pops. There is not a case
     * where 1 core pushes & pops while another core pushes & pops
     */
    if (IS_QUEUE_SHARED (pQueue) == TRUE) {
        if (Queue_EnterCritical (pQueue) != eSTATUS_SUCCESS) {
            return eSTATUS_FAILURE;
        }
        pQueue->count--;
        Queue_ExitCritical (pQueue);
    } else {
        pQueue->count--;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t Queue_Peek (Queue const* pQueue, void* pOutElement) {
    if (pQueue == NULL || pOutElement == NULL) {
        return eSTATUS_FAILURE;
    }

    if (QueueIsEmpty (pQueue)) {
        return eSTATUS_FAILURE;
    }

    // Calculate the memory location for the head element
    uint8_t const* pDataBytes = (uint8_t const*)pQueue->pData;
    uint8_t const* pHeadLocation =
    pDataBytes + (uint32_t)(pQueue->head * pQueue->elementSize);

    // Copy the element from the head position without removing it
    memcpy (pOutElement, pHeadLocation, pQueue->elementSize);

    return eSTATUS_SUCCESS;
}

void Queue_Clear (Queue* pQueue) {
    if (pQueue == NULL) {
        return;
    }

    pQueue->head  = 0;
    pQueue->tail  = 0;
    pQueue->count = 0;
}
