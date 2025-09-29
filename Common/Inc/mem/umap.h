#ifndef MEM_UMAP_H
#define MEM_UMAP_H

#include "common.h"

typedef bool (*UMapEqualFn_t) (void const* pKey1, void const* pKey2, uint16_t keySize);
typedef uint32_t (*UMapHashFn_t) (void const* pKey, uint32_t keySize);

typedef struct {
    void* pKey;
    void* pValue;
    bool occupied;
    uint32_t hash;
} UMapEntry_t;

typedef struct {
    uint32_t processID; // If 0 then umap is not shared
    UMapEntry_t* pEntries;
    uint16_t capacity;
    uint16_t keySize;
    uint16_t valueSize;
    uint16_t size;
    UMapHashFn_t hashFunc;
    UMapEqualFn_t equalFunc;
} UMap_t;

// Default hash function (djb2 algorithm)
uint32_t UMap_DefaultHash (void const* pKey, uint32_t keySize);

// Default equality function (memcmp)
bool UMap_DefaultEqual (void const* pKey1, void const* pKey2, uint32_t keySize);

bool UMap_Init (
UMap_t* pUMap,
UMapEntry_t* pEntries,
void* pKeyBuffer,
void* pValueBuffer,
uint16_t capacity,
uint16_t keySize,
uint16_t valueSize,
bool isShared
);

bool UMap_InitWithFunctions (
UMap_t* pUMap,
UMapEntry_t* pEntries,
void* pKeyBuffer,
void* pValueBuffer,
uint16_t capacity,
uint16_t keySize,
uint16_t valueSize,
bool isShared,
uint32_t (*hashFunc) (void const* pKey, uint16_t keySize),
bool (*equalFunc) (void const* pKey1, void const* pKey2, uint16_t keySize)
);

bool UMap_IsEmpty (UMap_t const* pUMap);
bool UMap_IsFull (UMap_t const* pUMap);
uint16_t UMap_Size (UMap_t const* pUMap);
uint16_t UMap_Capacity (UMap_t const* pUMap);

bool UMap_Insert (UMap_t* pUMap, void const* pKey, void const* pValue);
bool UMap_Find (UMap_t const* pUMap, void const* pKey, void* pOutValue);
void* UMap_FindPtr (UMap_t const* pUMap, void const* pKey);
bool UMap_Contains (UMap_t const* pUMap, void const* pKey);
bool UMap_Erase (UMap_t* pUMap, void const* pKey);
void UMap_Clear (UMap_t* pUMap);

float UMap_LoadFactor (UMap_t const* pUMap);

#define UMAP_FOR_EACH(pUMAP, KEY_TYPE, VALUE_TYPE, FN, ...) \
    do {                                                    \
        if ((pUMAP) == NULL || (FN) == NULL) {              \
            break;                                          \
        }                                                   \
        for (uint32_t i = 0; i < (pUMAP)->capacity; ++i) {  \
            if ((pUMAP)->pEntries[i].occupied) {            \
                FN (                                        \
                (KEY_TYPE*)(pUMAP)->pEntries[i].pKey,       \
                (VALUE_TYPE*)(pUMAP)->pEntries[i].pValue,   \
                ##__VA_ARGS__                               \
                );                                          \
            }                                               \
        }                                                   \
    } while (0)

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
    static inline bool NAME##UMap_Insert (KEY_TYPE const* pKey, VALUE_TYPE const* pValue) { \
        return UMap_Insert (&g_##NAME##_umap, pKey, pValue);                                \
    }                                                                                       \
    static inline bool NAME##UMap_InsertByValue (KEY_TYPE key, VALUE_TYPE value) {          \
        return UMap_Insert (&g_##NAME##_umap, &key, &value);                                \
    }                                                                                       \
    static inline bool NAME##UMap_Find (KEY_TYPE const* pKey, VALUE_TYPE* pOutValue) {      \
        return UMap_Find (&g_##NAME##_umap, pKey, pOutValue);                               \
    }                                                                                       \
    static inline VALUE_TYPE* NAME##UMap_FindPtr (KEY_TYPE const* pKey) {                   \
        return (VALUE_TYPE*)UMap_FindPtr (&g_##NAME##_umap, pKey);                          \
    }                                                                                       \
    static inline bool NAME##UMap_Contains (KEY_TYPE const* pKey) {                         \
        return UMap_Contains (&g_##NAME##_umap, pKey);                                      \
    }                                                                                       \
    static inline bool NAME##UMap_Erase (KEY_TYPE const* pKey) {                            \
        return UMap_Erase (&g_##NAME##_umap, pKey);                                         \
    }                                                                                       \
    static inline void NAME##UMap_Clear (void) {                                            \
        UMap_Clear (&g_##NAME##_umap);                                                      \
    }                                                                                       \
    static inline bool NAME##UMap_IsEmpty (void) {                                          \
        return UMap_IsEmpty (&g_##NAME##_umap);                                             \
    }                                                                                       \
    static inline bool NAME##UMap_IsFull (void) {                                           \
        return UMap_IsFull (&g_##NAME##_umap);                                              \
    }                                                                                       \
    static inline uint16_t NAME##UMap_Size (void) {                                         \
        return UMap_Size (&g_##NAME##_umap);                                                \
    }                                                                                       \
    static inline uint16_t NAME##UMap_Capacity (void) {                                     \
        return UMap_Capacity (&g_##NAME##_umap);                                            \
    }                                                                                       \
    static inline float NAME##UMap_LoadFactor (void) {                                      \
        return UMap_LoadFactor (&g_##NAME##_umap);                                          \
    }                                                                                       \
    static inline UMap_t* NAME##UMap_GetUMap (void) {                                       \
        return &g_##NAME##_umap;                                                            \
    }

#define UMAP_DEFINE_STATIC(NAME, KEY_TYPE, VALUE_TYPE, CAPACITY, IS_SHARED) \
    static UMapEntry_t g_##NAME##_entries[(CAPACITY)];                      \
    static KEY_TYPE g_##NAME##_keys[(CAPACITY)];                            \
    static VALUE_TYPE g_##NAME##_values[(CAPACITY)];                        \
    static UMap_t g_##NAME##_umap = { 0 };                                  \
    UMAP_DEFINE_FUNCTIONS_ONLY (NAME, KEY_TYPE, VALUE_TYPE, CAPACITY, IS_SHARED)

#define UMAP_DEFINE_STATIC_SHARED(NAME, KEY_TYPE, VALUE_TYPE, CAPACITY)   \
    static SHARED_MEM_SECTION UMapEntry_t g_##NAME##_entries[(CAPACITY)]; \
    static SHARED_MEM_SECTION KEY_TYPE g_##NAME##_keys[(CAPACITY)];       \
    static SHARED_MEM_SECTION VALUE_TYPE g_##NAME##_values[(CAPACITY)];   \
    static SHARED_MEM_SECTION UMap_t g_##NAME##_umap = { 0 };             \
    UMAP_DEFINE_FUNCTIONS_ONLY (NAME, KEY_TYPE, VALUE_TYPE, CAPACITY, true)

#endif // MEM_UMAP_H