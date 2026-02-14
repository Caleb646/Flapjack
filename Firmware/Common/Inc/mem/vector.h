#ifndef MEM_VECTOR_H
#define MEM_VECTOR_H

#include "core/core.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t processID; // If 0 then vector is not shared
    void* pData;
    uint16_t capacity;
    uint16_t elementSize;
    uint16_t size;
} Vector_t;

eSTATUS_t Vector_Init (Vector_t* pVector, void* pBuffer, uint16_t capacity, uint16_t elementSize, bool isShared);
bool Vector_IsEmpty (Vector_t const* pVector);
bool Vector_IsFull (Vector_t const* pVector);
uint16_t Vector_Size (Vector_t const* pVector);
uint16_t Vector_Capacity (Vector_t const* pVector);
eSTATUS_t Vector_PushBack (Vector_t* pVector, void const* pElement);
eSTATUS_t Vector_PopBack (Vector_t* pVector, void* pOutElement);
void* Vector_At (Vector_t const* pVector, uint16_t index);
void* Vector_Front (Vector_t const* pVector);
void* Vector_Back (Vector_t const* pVector);
eSTATUS_t Vector_Insert (Vector_t* pVector, uint16_t index, void const* pElement);
eSTATUS_t Vector_Erase (Vector_t* pVector, uint16_t index, void* pOutElement);
void Vector_Clear (Vector_t* pVector);
eSTATUS_t Vector_Resize (Vector_t* pVector, uint16_t newSize, void const* pFillValue);
void* Vector_Data (Vector_t const* pVector);

#define VECTOR_FOR_EACH(pVECTOR, TYPE, FN, ...)                  \
    do {                                                         \
        if ((pVECTOR) == NULL || (FN) == NULL) {                 \
            break;                                               \
        }                                                        \
        for (uint32_t i = 0; i < (pVECTOR)->size; ++i) {         \
            FN ((TYPE*)Vector_At ((pVECTOR), i), ##__VA_ARGS__); \
        }                                                        \
    } while (0)

#define VECTOR_DEFINE_FUNCTIONS_ONLY(NAME, TYPE, CAPACITY, IS_SHARED)                                       \
    static inline eSTATUS_t NAME##Vector_Init (void) {                                                      \
        return Vector_Init (&g_##NAME##_vector, g_##NAME##_buffer, (CAPACITY), sizeof (TYPE), (IS_SHARED)); \
    }                                                                                                       \
    static inline eSTATUS_t NAME##Vector_PushBack (TYPE const* pElement) {                                  \
        return Vector_PushBack (&g_##NAME##_vector, pElement);                                              \
    }                                                                                                       \
    static inline eSTATUS_t NAME##Vector_PopBack (TYPE* pOutElement) {                                      \
        return Vector_PopBack (&g_##NAME##_vector, pOutElement);                                            \
    }                                                                                                       \
    static inline TYPE* NAME##Vector_At (uint16_t index) {                                                  \
        return (TYPE*)Vector_At (&g_##NAME##_vector, index);                                                \
    }                                                                                                       \
    static inline TYPE* NAME##Vector_Front (void) {                                                         \
        return (TYPE*)Vector_Front (&g_##NAME##_vector);                                                    \
    }                                                                                                       \
    static inline TYPE* NAME##Vector_Back (void) {                                                          \
        return (TYPE*)Vector_Back (&g_##NAME##_vector);                                                     \
    }                                                                                                       \
    static inline eSTATUS_t NAME##Vector_Insert (uint16_t index, TYPE const* pElement) {                    \
        return Vector_Insert (&g_##NAME##_vector, index, pElement);                                         \
    }                                                                                                       \
    static inline eSTATUS_t NAME##Vector_Erase (uint16_t index, TYPE* pOutElement) {                        \
        return Vector_Erase (&g_##NAME##_vector, index, pOutElement);                                       \
    }                                                                                                       \
    static inline void NAME##Vector_Clear (void) {                                                          \
        Vector_Clear (&g_##NAME##_vector);                                                                  \
    }                                                                                                       \
    static inline eSTATUS_t NAME##Vector_Resize (uint16_t newSize, TYPE const* pFillValue) {                \
        return Vector_Resize (&g_##NAME##_vector, newSize, pFillValue);                                     \
    }                                                                                                       \
    static inline bool NAME##Vector_IsFull (void) {                                                         \
        return Vector_IsFull (&g_##NAME##_vector);                                                          \
    }                                                                                                       \
    static inline bool NAME##Vector_IsEmpty (void) {                                                        \
        return Vector_IsEmpty (&g_##NAME##_vector);                                                         \
    }                                                                                                       \
    static inline uint16_t NAME##Vector_Size (void) {                                                       \
        return Vector_Size (&g_##NAME##_vector);                                                            \
    }                                                                                                       \
    static inline uint16_t NAME##Vector_Capacity (void) {                                                   \
        return Vector_Capacity (&g_##NAME##_vector);                                                        \
    }                                                                                                       \
    static inline TYPE* NAME##Vector_Data (void) {                                                          \
        return (TYPE*)Vector_Data (&g_##NAME##_vector);                                                     \
    }                                                                                                       \
    static inline Vector_t* NAME##Vector_GetVector (void) {                                                 \
        return &g_##NAME##_vector;                                                                          \
    }

#define VECTOR_DEFINE_STATIC(NAME, TYPE, CAPACITY, IS_SHARED) \
    static TYPE g_##NAME##_buffer[(CAPACITY)];                \
    static Vector_t g_##NAME##_vector = { 0 };                \
    VECTOR_DEFINE_FUNCTIONS_ONLY (NAME, TYPE, CAPACITY, IS_SHARED)

#define VECTOR_DEFINE_STATIC_SHARED(NAME, TYPE, CAPACITY)             \
    static SHARED_MEM_BSS_SECTION TYPE g_##NAME##_buffer[(CAPACITY)]; \
    static SHARED_MEM_BSS_SECTION Vector_t g_##NAME##_vector = { 0 }; \
    VECTOR_DEFINE_FUNCTIONS_ONLY (NAME, TYPE, CAPACITY, true)

#endif // MEM_VECTOR_H