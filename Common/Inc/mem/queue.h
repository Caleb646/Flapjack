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
} Queue_t;

eSTATUS_t
QueueInit (Queue_t* pQueue, void* pBuffer, uint16_t capacity, uint16_t elementSize, bool isShared);
bool QueueIsEmpty (Queue_t const* pQueue);
bool QueueIsFull (Queue_t const* pQueue);
uint16_t QueueGetElementCount (Queue_t const* pQueue);
uint16_t QueueGetCapacity (Queue_t const* pQueue);
eSTATUS_t Queue_Push (Queue_t* pQueue, void const* pElement);
eSTATUS_t Queue_Pop (Queue_t* pQueue, void* pOutElement);
eSTATUS_t Queue_Peek (Queue_t const* pQueue, void* pOutElement);
void Queue_Clear (Queue_t* pQueue);

#define QUEUE_DEFINE_FUNCTIONS_ONLY(NAME, TYPE, CAPACITY, IS_SHARED)                                     \
    static inline eSTATUS_t NAME##Queue_Init (void) {                                                    \
        return QueueInit (&g_##NAME##_queue, g_##NAME##_buffer, (CAPACITY), sizeof (TYPE), (IS_SHARED)); \
    }                                                                                                    \
    static inline eSTATUS_t NAME##Queue_Push (TYPE const* pElement) {                                    \
        return Queue_Push (&g_##NAME##_queue, pElement);                                                 \
    }                                                                                                    \
    static inline eSTATUS_t NAME##Queue_Pop (TYPE* pOutElement) {                                        \
        return Queue_Pop (&g_##NAME##_queue, pOutElement);                                               \
    }                                                                                                    \
    static inline eSTATUS_t NAME##Queue_Peek (TYPE* pOutElement) {                                       \
        return Queue_Peek (&g_##NAME##_queue, pOutElement);                                              \
    }                                                                                                    \
    static inline eSTATUS_t NAME##Queue_IsFull (void) {                                                  \
        return QueueIsFull (&g_##NAME##_queue);                                                          \
    }                                                                                                    \
    static inline eSTATUS_t NAME##Queue_IsEmpty (void) {                                                 \
        return QueueIsEmpty (&g_##NAME##_queue);                                                         \
    }

#define QUEUE_DEFINE_STATIC(NAME, TYPE, CAPACITY, IS_SHARED) \
    static TYPE g_##NAME##_buffer[(CAPACITY)];               \
    static Queue_t g_##NAME##_queue = { 0 };                 \
    QUEUE_DEFINE_FUNCTIONS_ONLY (NAME, TYPE, CAPACITY, IS_SHARED)

#define QUEUE_DEFINE_STATIC_SHARED(NAME, TYPE, CAPACITY)          \
    static SHARED_MEM_SECTION TYPE g_##NAME##_buffer[(CAPACITY)]; \
    static SHARED_MEM_SECTION Queue_t g_##NAME##_queue = { 0 };   \
    QUEUE_DEFINE_FUNCTIONS_ONLY (NAME, TYPE, CAPACITY, true)

#endif // MEM_QUEUE_H
