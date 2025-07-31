#ifndef MEM_QUEUE_H
#define MEM_QUEUE_H

#include "common.h"
#include <stdint.h>

/**
 * @brief Generic circular queue implementation
 *
 * This queue can store any type of data up to the specified element size.
 * It uses a circular buffer internally for efficient memory usage.
 */

typedef struct {
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
eSTATUS_t QueueInit (Queue* pQueue, void* pBuffer, uint16_t capacity, uint16_t elementSize);

/**
 * @brief Check if the queue is empty
 *
 * @param queue Pointer to the queue structure
 * @return uint8_t 1 if empty, 0 if not empty
 */
uint8_t QueueIsEmpty (Queue const* pQueue);

/**
 * @brief Check if the queue is full
 *
 * @param queue Pointer to the queue structure
 * @return uint8_t 1 if full, 0 if not full
 */
uint8_t QueueIsFull (Queue const* pQueue);

/**
 * @brief Get the current number of elements in the queue
 *
 * @param queue Pointer to the queue structure
 * @return uint16_t Number of elements currently in the queue
 */
uint16_t QueueCount (Queue const* pQueue);

/**
 * @brief Get the maximum capacity of the queue
 *
 * @param queue Pointer to the queue structure
 * @return uint16_t Maximum number of elements the queue can hold
 */
uint16_t QueueCapacity (Queue const* pQueue);

/**
 * @brief Add an element to the rear of the queue
 *
 * @param queue Pointer to the queue structure
 * @param element Pointer to the element to be added
 * @return eSTATUS_t eSTATUS_SUCCESS on success, eSTATUS_FAILURE if queue is full
 */
eSTATUS_t QueueEnqueue (Queue* pQueue, void const* pElement);

/**
 * @brief Remove and return an element from the front of the queue
 *
 * @param queue Pointer to the queue structure
 * @param out_element Pointer to store the dequeued element
 * @return eSTATUS_t eSTATUS_SUCCESS on success, eSTATUS_FAILURE if queue is empty
 */
eSTATUS_t QueueDequeue (Queue* pQueue, void* pOutElement);

/**
 * @brief Get the front element without removing it from the queue
 *
 * @param queue Pointer to the queue structure
 * @param out_element Pointer to store the front element
 * @return eSTATUS_t eSTATUS_SUCCESS on success, eSTATUS_FAILURE if queue is empty
 */
eSTATUS_t QueuePeek (Queue const* pQueue, void* pOutElement);

/**
 * @brief Clear all elements from the queue
 *
 * @param queue Pointer to the queue structure
 */
void QueueClear (Queue* pQueue);

/**
 * @brief Macro to declare a statically allocated queue
 *
 * @param name Name of the queue variable
 * @param type Type of elements to store
 * @param capacity Maximum number of elements
 */
#define QUEUE_DECLARE_STATIC(name, type, capacity)                        \
    static type name##_buffer[capacity];                                  \
    static Queue name = { 0 };                                            \
    static inline eSTATUS_t name##_Init (void) {                          \
        return QueueInit (&name, name##_buffer, capacity, sizeof (type)); \
    }

/**
 * @brief Macro to create queue operation functions for a specific type
 *
 * @param name Name prefix for the functions
 * @param type Type of elements to store
 */
#define QUEUE_DEFINE_TYPE_SAFE(name, type)                                        \
    static inline eSTATUS_t name##_Enqueue (Queue* queue, const type* element) {  \
        return QueueEnqueue (queue, element);                                     \
    }                                                                             \
    static inline eSTATUS_t name##_Dequeue (Queue* queue, type* out_element) {    \
        return QueueDequeue (queue, out_element);                                 \
    }                                                                             \
    static inline eSTATUS_t name##_Peek (const Queue* queue, type* out_element) { \
        return QueuePeek (queue, out_element);                                    \
    }

#endif // MEM_QUEUE_H
