#ifndef MEM_UMAP_H
#define MEM_UMAP_H

#include "core/core.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool occupied;
    uint32_t hash;
} UMapEntry_t;

typedef struct {
    uint32_t processID;
    UMapEntry_t* pEntries;
    void* pKeys;
    void* pValues;
    uint16_t capacity;
    uint16_t size;
    uint16_t keySize;
    uint16_t valueSize;
    uint32_t (*hashFunc) (void const* pKey, uint16_t keySize);
    bool (*equalFunc) (void const* pKey1, void const* pKey2, uint16_t keySize);
} UMap_t;

bool UMap_Init (UMap_t* pUMap, UMapEntry_t* pEntries, void* pKeys, void* pValues, uint16_t capacity, uint16_t keySize, uint16_t valueSize, bool isShared);

bool UMap_InitWithFunctions (
UMap_t* pUMap,
UMapEntry_t* pEntries,
void* pKeys,
void* pValues,
uint16_t capacity,
uint16_t keySize,
uint16_t valueSize,
bool isShared,
uint32_t (*hashFunc) (void const*, uint16_t),
bool (*equalFunc) (void const*, void const*, uint16_t)
);

bool UMap_Insert (UMap_t* pUMap, void const* pKey, void const* pValue);
bool UMap_Find (UMap_t const* pUMap, void const* pKey, void* pOutValue);
void* UMap_FindPtr (UMap_t const* pUMap, void const* pKey);
bool UMap_Contains (UMap_t const* pUMap, void const* pKey);
void UMap_Clear (UMap_t* pUMap);
bool UMap_IsEmpty (UMap_t const* pUMap);
bool UMap_IsFull (UMap_t const* pUMap);
uint16_t UMap_Size (UMap_t const* pUMap);
uint16_t UMap_Capacity (UMap_t const* pUMap);
float UMap_LoadFactor (UMap_t const* pUMap);

uint32_t UMap_DefaultHash (void const* pKey, uint16_t keySize);
bool UMap_DefaultEqual (void const* pKey1, void const* pKey2, uint16_t keySize);

#define UMAP_DEFINE_FUNCTIONS_ONLY(NAME, KEY_TYPE, VALUE_TYPE, CAPACITY, IS_SHARED)         \
    static inline bool NAME##UMap_Init (void) {                                             \
        return UMap_Init (                                                                  \
        &g_##NAME##_umap,                                                                   \
        g_##NAME##_entries,                                                                 \
        g_##NAME##_keys,                                                                    \
        g_##NAME##_values,                                                                  \
        (CAPACITY),                                                                         \
        sizeof (KEY_TYPE),                                                                  \
        sizeof (VALUE_TYPE),                                                                \
        (IS_SHARED)                                                                         \
        );                                                                                  \
    }                                                                                       \
                                                                                            \
    static inline bool NAME##UMap_InitWithFunctions (                                       \
    uint32_t (*hashFunc) (void const*, uint16_t),                                           \
    bool (*equalFunc) (void const*, void const*, uint16_t)                                  \
    ) {                                                                                     \
        return UMap_InitWithFunctions (                                                     \
        &g_##NAME##_umap,                                                                   \
        g_##NAME##_entries,                                                                 \
        g_##NAME##_keys,                                                                    \
        g_##NAME##_values,                                                                  \
        (CAPACITY),                                                                         \
        sizeof (KEY_TYPE),                                                                  \
        sizeof (VALUE_TYPE),                                                                \
        (IS_SHARED),                                                                        \
        hashFunc,                                                                           \
        equalFunc                                                                           \
        );                                                                                  \
    }                                                                                       \
                                                                                            \
    static inline bool NAME##UMap_Insert (KEY_TYPE const* pKey, VALUE_TYPE const* pValue) { \
        return UMap_Insert (&g_##NAME##_umap, pKey, pValue);                                \
    }                                                                                       \
                                                                                            \
    static inline bool NAME##UMap_InsertByValue (KEY_TYPE key, VALUE_TYPE value) {          \
        return UMap_Insert (&g_##NAME##_umap, &key, &value);                                \
    }                                                                                       \
                                                                                            \
    static inline bool NAME##UMap_Find (KEY_TYPE const* pKey, VALUE_TYPE* pOutValue) {      \
        return UMap_Find (&g_##NAME##_umap, pKey, pOutValue);                               \
    }                                                                                       \
                                                                                            \
    static inline VALUE_TYPE* NAME##UMap_FindPtr (KEY_TYPE const* pKey) {                   \
        return (VALUE_TYPE*)UMap_FindPtr (&g_##NAME##_umap, pKey);                          \
    }                                                                                       \
                                                                                            \
    static inline bool NAME##UMap_Contains (KEY_TYPE const* pKey) {                         \
        return UMap_Contains (&g_##NAME##_umap, pKey);                                      \
    }                                                                                       \
                                                                                            \
    static inline void NAME##UMap_Clear (void) {                                            \
        UMap_Clear (&g_##NAME##_umap);                                                      \
    }                                                                                       \
                                                                                            \
    static inline bool NAME##UMap_IsEmpty (void) {                                          \
        return UMap_IsEmpty (&g_##NAME##_umap);                                             \
    }                                                                                       \
                                                                                            \
    static inline bool NAME##UMap_IsFull (void) {                                           \
        return UMap_IsFull (&g_##NAME##_umap);                                              \
    }                                                                                       \
                                                                                            \
    static inline uint16_t NAME##UMap_Size (void) {                                         \
        return UMap_Size (&g_##NAME##_umap);                                                \
    }                                                                                       \
                                                                                            \
    static inline uint16_t NAME##UMap_Capacity (void) {                                     \
        return UMap_Capacity (&g_##NAME##_umap);                                            \
    }                                                                                       \
                                                                                            \
    static inline float NAME##UMap_LoadFactor (void) {                                      \
        return UMap_LoadFactor (&g_##NAME##_umap);                                          \
    }                                                                                       \
                                                                                            \
    static inline UMap_t* NAME##UMap_GetUMap (void) {                                       \
        return &g_##NAME##_umap;                                                            \
    }

#define UMAP_DEFINE_STATIC(NAME, KEY_TYPE, VALUE_TYPE, CAPACITY, IS_SHARED) \
    static KEY_TYPE g_##NAME##_keys[(CAPACITY)];                            \
    static VALUE_TYPE g_##NAME##_values[(CAPACITY)];                        \
    static UMapEntry_t g_##NAME##_entries[(CAPACITY)];                      \
    static UMap_t g_##NAME##_umap = { 0 };                                  \
    UMAP_DEFINE_FUNCTIONS_ONLY (NAME, KEY_TYPE, VALUE_TYPE, CAPACITY, IS_SHARED)


#define UMAP_DEFINE_STATIC_SHARED(NAME, KEY_TYPE, VALUE_TYPE, CAPACITY)       \
    static SHARED_MEM_BSS_SECTION KEY_TYPE g_##NAME##_keys[(CAPACITY)];       \
    static SHARED_MEM_BSS_SECTION VALUE_TYPE g_##NAME##_values[(CAPACITY)];   \
    static SHARED_MEM_BSS_SECTION UMapEntry_t g_##NAME##_entries[(CAPACITY)]; \
    static SHARED_MEM_BSS_SECTION UMap_t g_##NAME##_umap = { 0 };             \
    UMAP_DEFINE_FUNCTIONS_ONLY (NAME, KEY_TYPE, VALUE_TYPE, CAPACITY, true)

#define UMAP_FOR_EACH(pUMAP, KEY_TYPE, VALUE_TYPE, FN, ...) \
    do {                                                    \
        if ((pUMAP) == NULL || (FN) == NULL)                \
            break;                                          \
        KEY_TYPE* keys     = (KEY_TYPE*)(pUMAP)->pKeys;     \
        VALUE_TYPE* values = (VALUE_TYPE*)(pUMAP)->pValues; \
        for (uint32_t i = 0; i < (pUMAP)->capacity; ++i) {  \
            if ((pUMAP)->pEntries[i].occupied) {            \
                FN (&keys[i], &values[i], ##__VA_ARGS__);   \
            }                                               \
        }                                                   \
    } while (0)

#endif // MEM_UMAP_H