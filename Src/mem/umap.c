#include "mem/umap.h"
#include "core/core.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>


static inline void* UMap_GetKey (UMap_t const* pUMap, uint16_t index) {
    return ((uint8_t*)pUMap->pKeys) + (index * pUMap->keySize);
}

static inline void* UMap_GetValue (UMap_t const* pUMap, uint16_t index) {
    return ((uint8_t*)pUMap->pValues) + (index * pUMap->valueSize);
}

static uint16_t UMap_FindIndex (UMap_t const* pUMap, void const* pKey, uint32_t hash) {

    uint16_t index         = hash % pUMap->capacity;
    uint16_t originalIndex = index;

    while (pUMap->pEntries[index].occupied) {
        if (pUMap->pEntries[index].hash == hash &&
            pUMap->equalFunc (UMap_GetKey (pUMap, index), pKey, pUMap->keySize)) {
            return index;
        }
        index = (index + 1) % pUMap->capacity;
        if (index == originalIndex) {
            break;
        }
    }

    return index;
}

uint32_t UMap_DefaultHash (void const* pKey, uint16_t keySize) {

    if (pKey == NULL || keySize == 0) {
        return 0;
    }

    uint32_t hash         = 5381;
    uint8_t const* pBytes = (uint8_t const*)pKey;

    for (uint16_t i = 0; i < keySize; ++i) {
        hash = ((hash << 5U) + hash) + pBytes[i];
    }

    return hash;
}

bool UMap_DefaultEqual (void const* pKey1, void const* pKey2, uint16_t keySize) {

    if (pKey1 == NULL || pKey2 == NULL) {
        return false;
    }

    return memcmp (pKey1, pKey2, keySize) == 0U;
}

bool UMap_Init (UMap_t* pUMap, UMapEntry_t* pEntries, void* pKeys, void* pValues, uint16_t capacity, uint16_t keySize, uint16_t valueSize, bool isShared) {
    return UMap_InitWithFunctions (pUMap, pEntries, pKeys, pValues, capacity, keySize, valueSize, isShared, UMap_DefaultHash, UMap_DefaultEqual);
}

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
) {
    if (pUMap == NULL || pKeys == NULL || pValues == NULL || pEntries == NULL || capacity == 0 ||
        keySize == 0 || valueSize == 0 || hashFunc == NULL || equalFunc == NULL) {
        return false;
    }

    memset (pUMap, 0, sizeof (UMap_t));
    memset (pEntries, 0, sizeof (UMapEntry_t) * capacity);
    memset (pKeys, 0, (size_t)keySize * capacity);
    memset (pValues, 0, (size_t)valueSize * capacity);

    pUMap->processID = isShared ? 1 : 0;
    pUMap->pKeys     = pKeys;
    pUMap->pValues   = pValues;
    pUMap->pEntries  = pEntries;
    pUMap->capacity  = capacity;
    pUMap->size      = 0;
    pUMap->keySize   = keySize;
    pUMap->valueSize = valueSize;
    pUMap->hashFunc  = hashFunc;
    pUMap->equalFunc = equalFunc;

    return true;
}

bool UMap_Insert (UMap_t* pUMap, void const* pKey, void const* pValue) {

    if (pUMap == NULL || pKey == NULL || pValue == NULL) {
        return false;
    }

    if (UMap_IsFull (pUMap)) {
        return false;
    }

    uint32_t hash  = pUMap->hashFunc (pKey, pUMap->keySize);
    uint16_t index = UMap_FindIndex (pUMap, pKey, hash);

    if (pUMap->pEntries[index].occupied == false) {
        ++pUMap->size;
    }

    memcpy (UMap_GetKey (pUMap, index), pKey, pUMap->keySize);
    memcpy (UMap_GetValue (pUMap, index), pValue, pUMap->valueSize);

    pUMap->pEntries[index].occupied = true;
    pUMap->pEntries[index].hash     = hash;

    return true;
}

bool UMap_Find (UMap_t const* pUMap, void const* pKey, void* pOutValue) {

    if (pUMap == NULL || pKey == NULL || pOutValue == NULL) {
        return false;
    }

    uint32_t hash  = pUMap->hashFunc (pKey, pUMap->keySize);
    uint16_t index = UMap_FindIndex (pUMap, pKey, hash);

    if (pUMap->pEntries[index].occupied && pUMap->pEntries[index].hash == hash &&
        pUMap->equalFunc (UMap_GetKey (pUMap, index), pKey, pUMap->keySize)) {
        memcpy (pOutValue, UMap_GetValue (pUMap, index), pUMap->valueSize);
        return true;
    }

    return false;
}

void* UMap_FindPtr (UMap_t const* pUMap, void const* pKey) {

    if (pUMap == NULL || pKey == NULL) {
        return NULL;
    }

    uint32_t hash  = pUMap->hashFunc (pKey, pUMap->keySize);
    uint16_t index = UMap_FindIndex (pUMap, pKey, hash);

    if (pUMap->pEntries[index].occupied && pUMap->pEntries[index].hash == hash &&
        pUMap->equalFunc (UMap_GetKey (pUMap, index), pKey, pUMap->keySize)) {
        return UMap_GetValue (pUMap, index);
    }

    return NULL;
}

bool UMap_Contains (UMap_t const* pUMap, void const* pKey) {
    return UMap_FindPtr (pUMap, pKey) != NULL;
}

void UMap_Clear (UMap_t* pUMap) {

    if (pUMap == NULL) {
        return;
    }

    memset (pUMap->pEntries, 0, sizeof (UMapEntry_t) * pUMap->capacity);
    memset (pUMap->pKeys, 0, (size_t)pUMap->keySize * pUMap->capacity);
    memset (pUMap->pValues, 0, (size_t)pUMap->valueSize * pUMap->capacity);
    pUMap->size = 0;
}

bool UMap_IsEmpty (UMap_t const* pUMap) {
    return pUMap == NULL ? true : pUMap->size == 0U;
}

bool UMap_IsFull (UMap_t const* pUMap) {
    return pUMap == NULL ? true : pUMap->size >= pUMap->capacity;
}

uint16_t UMap_Size (UMap_t const* pUMap) {
    return pUMap == NULL ? 0 : pUMap->size;
}

uint16_t UMap_Capacity (UMap_t const* pUMap) {
    return pUMap == NULL ? 0 : pUMap->capacity;
}

float UMap_LoadFactor (UMap_t const* pUMap) {

    if (pUMap == NULL || pUMap->capacity == 0) {
        return 1.0F;
    }
    return (float)pUMap->size / (float)pUMap->capacity;
}