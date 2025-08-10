#ifndef MEM_QUEUE_H
#define MEM_QUEUE_H

#include "common.h"
#include <stdint.h>


typedef struct {
    uint32_t processID;   // If 0 then queue is not a SharedQueue
    void* pData;          // Pointer to the data buffer
    uint16_t capacity;    // Maximum number of elements
    uint16_t elementSize; // Size of each element in bytes
    uint16_t head;        // Index of the first element
    uint16_t tail;        // Index where next element will be inserted
    uint16_t count;       // Current number of elements
} Queue;

/**
 * @brief Initialize a queue with the given parameters
 *
 * @param pData Pointer to the queue structure
 * @param pBuffer Pointer to the memory buffer for storing elements
 * @param capacity Maximum number of elements the queue can hold
 * @param elementSize Size of each element in bytes
 * @return eSTATUS_t eSTATUS_SUCCESS on success, eSTATUS_FAILURE on failure
 */
eSTATUS_t
QueueInit (Queue* pQueue, void* pBuffer, uint16_t capacity, uint16_t elementSize, BOOL_t isShared);
eSTATUS_t
QueueInit_SharedMemory (void* pMemoryStart, uint32_t memorySize, uint16_t capacity, uint16_t elementSize);

/**
 * @brief Check if the queue is empty
 *
 * @param queue Pointer to the queue structure
 * @return BOOL_t TRUE if empty, FALSE if not empty
 */
BOOL_t QueueIsEmpty (Queue const* pQueue);

/**
 * @brief Check if the queue is full
 *
 * @param queue Pointer to the queue structure
 * @return BOOL_t TRUE if full, FALSE if not full
 */
BOOL_t QueueIsFull (Queue const* pQueue);

/**
 * @brief Get the current number of elements in the queue
 *
 * @param queue Pointer to the queue structure
 * @return uint16_t Number of elements currently in the queue
 */
uint16_t QueueGetElementCount (Queue const* pQueue);

/**
 * @brief Get the maximum capacity of the queue
 *
 * @param queue Pointer to the queue structure
 * @return uint16_t Maximum number of elements the queue can hold
 */
uint16_t QueueGetCapacity (Queue const* pQueue);

/**
 * @brief Add an element to the rear of the queue
 *
 * @param queue Pointer to the queue structure
 * @param element Pointer to the element to be added
 * @return eSTATUS_t eSTATUS_SUCCESS on success, eSTATUS_FAILURE if queue is full
 */
eSTATUS_t Queue_Push (Queue* pQueue, void const* pElement);

/**
 * @brief Remove and return an element from the front of the queue
 *
 * @param queue Pointer to the queue structure
 * @param out_element Pointer to store the dequeued element
 * @return eSTATUS_t eSTATUS_SUCCESS on success, eSTATUS_FAILURE if queue is empty
 */
eSTATUS_t Queue_Pop (Queue* pQueue, void* pOutElement);

/**
 * @brief Get the front element without removing it from the queue
 *
 * @param queue Pointer to the queue structure
 * @param out_element Pointer to store the front element
 * @return eSTATUS_t eSTATUS_SUCCESS on success, eSTATUS_FAILURE if queue is empty
 */
eSTATUS_t Queue_Peek (Queue const* pQueue, void* pOutElement);

/**
 * @brief Clear all elements from the queue
 *
 * @param queue Pointer to the queue structure
 */
void Queue_Clear (Queue* pQueue);

#define QUEUE_DEFINE_STATIC_SHARED_MEMORY(NAME, TYPE, pMemoryStart, memorySize, capacity) \
    static Queue* gp_##NAME##_queue = (pMemoryStart);                                     \
    static inline eSTATUS_t NAME##Queue_Init (void) {                                     \
        return QueueInit_SharedMemory (                                                   \
        gp_##NAME##_queue, (memorySize), (capacity), sizeof (TYPE));                      \
    }                                                                                     \
    static inline eSTATUS_t NAME##Queue_Push (TYPE const* pElement) {                     \
        return Queue_Push (gp_##NAME##_queue, pElement);                                  \
    }                                                                                     \
    static inline eSTATUS_t NAME##Queue_Pop (TYPE* pOutElement) {                         \
        return Queue_Pop (gp_##NAME##_queue, pOutElement);                                \
    }                                                                                     \
    static inline eSTATUS_t NAME##Queue_Peek (TYPE* pOutElement) {                        \
        return Queue_Peek (gp_##NAME##_queue, pOutElement);                               \
    }                                                                                     \
    static inline eSTATUS_t NAME##Queue_IsFull (void) {                                   \
        return QueueIsFull (gp_##NAME##_queue);                                           \
    }                                                                                     \
    static inline eSTATUS_t NAME##Queue_IsEmpty (void) {                                  \
        return QueueIsEmpty (gp_##NAME##_queue);                                          \
    }

#define QUEUE_DEFINE_STATIC(NAME, TYPE, capacity, isShared)                           \
    static TYPE g_##NAME##_buffer[(capacity)];                                        \
    static Queue g_##NAME##_queue = { 0 };                                            \
    static inline eSTATUS_t NAME##Queue_Init (void) {                                 \
        return QueueInit (                                                            \
        &g_##NAME##_queue, g_##NAME##_buffer, (capacity), sizeof (TYPE), (isShared)); \
    }                                                                                 \
    static inline eSTATUS_t NAME##Queue_Push (TYPE const* pElement) {                 \
        return Queue_Push (&g_##NAME##_queue, pElement);                              \
    }                                                                                 \
    static inline eSTATUS_t NAME##Queue_Pop (TYPE* pOutElement) {                     \
        return Queue_Pop (&g_##NAME##_queue, pOutElement);                            \
    }                                                                                 \
    static inline eSTATUS_t NAME##Queue_Peek (TYPE* pOutElement) {                    \
        return Queue_Peek (&g_##NAME##_queue, pOutElement);                           \
    }                                                                                 \
    static inline eSTATUS_t NAME##Queue_IsFull (void) {                               \
        return QueueIsFull (&g_##NAME##_queue);                                       \
    }                                                                                 \
    static inline eSTATUS_t NAME##Queue_IsEmpty (void) {                              \
        return QueueIsEmpty (&g_##NAME##_queue);                                      \
    }

#endif // MEM_QUEUE_H
