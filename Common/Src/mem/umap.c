#include "mem/umap.h"
#include "common.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static uint16_t UMap_FindIndex (UMap_t const* pUMap, void const* pKey, uint32_t hash) {

    uint16_t index         = hash % pUMap->capacity;
    uint16_t originalIndex = index;

    // Linear probing
    while (pUMap->pEntries[index].occupied) {
        if (pUMap->pEntries[index].hash == hash &&
            pUMap->equalFunc (pUMap->pEntries[index].pKey, pKey, pUMap->keySize)) {
            return index; // Found existing key
        }

        index = (index + 1) % pUMap->capacity;

        // If we've wrapped around, the table is full
        if (index == originalIndex) {
            break;
        }
    }

    return index; // Return empty slot or wrapped-around position
}

uint32_t UMap_DefaultHash (void const* pKey, size_t keySize) {

    if (pKey == NULL || keySize == 0) {
        return 0;
    }

    uint32_t hash         = 5381;
    uint8_t const* pBytes = (uint8_t const*)pKey;

    for (uint32_t i = 0; i < keySize; ++i) {
        hash = ((hash << 5U) + hash) + pBytes[i]; // hash * 33 + c
    }

    return hash;
}

bool UMap_DefaultEqual (void const* pKey1, void const* pKey2, size_t keySize) {

    if (pKey1 == NULL || pKey2 == NULL) {
        return false;
    }

    return memcmp (pKey1, pKey2, keySize);
}

bool UMap_Init (
UMap_t* pUMap,
UMapEntry_t* pEntries,
void* pKeyBuffer,
void* pValueBuffer,
uint16_t capacity,
uint16_t keySize,
uint16_t valueSize,
bool isShared
) {

    return UMap_InitWithFunctions (pUMap, pEntries, pKeyBuffer, pValueBuffer, capacity, keySize, valueSize, isShared, UMap_DefaultHash, UMap_DefaultEqual);
}

bool UMap_InitWithFunctions (
UMap_t* pUMap,
UMapEntry_t* pEntries,
void* pKeyBuffer,
void* pValueBuffer,
uint16_t capacity,
uint16_t keySize,
uint16_t valueSize,
bool isShared,
UMapHashFn_t hashFunc,
UMapEqualFn_t equalFunc
) {

    if (pUMap == NULL || pEntries == NULL || pKeyBuffer == NULL ||
        pValueBuffer == NULL || capacity == 0 || keySize == 0 ||
        valueSize == 0 || hashFunc == NULL || equalFunc == NULL) {
        return false;
    }

    memset ((void*)pUMap, 0, sizeof (UMap_t));
    memset ((void*)pEntries, 0, sizeof (UMapEntry_t) * (size_t)capacity);
    pUMap->processID = isShared ? 1 : 0;
    pUMap->pEntries  = pEntries;
    pUMap->capacity  = capacity;
    pUMap->keySize   = keySize;
    pUMap->valueSize = valueSize;
    pUMap->size      = 0;
    pUMap->hashFunc  = hashFunc;
    pUMap->equalFunc = equalFunc;
    return true;
}

bool UMap_IsEmpty (UMap_t const* pUMap) {

    if (pUMap == NULL) {
        return true;
    }
    return pUMap->size == 0;
}

bool UMap_IsFull (UMap_t const* pUMap) {

    if (pUMap == NULL) {
        return true;
    }
    return pUMap->size >= pUMap->capacity;
}

uint16_t UMap_Size (UMap_t const* pUMap) {

    if (pUMap == NULL) {
        return 0;
    }
    return pUMap->size;
}

uint16_t UMap_Capacity (UMap_t const* pUMap) {

    if (pUMap == NULL) {
        return 0;
    }
    return pUMap->capacity;
}

float UMap_LoadFactor (UMap_t const* pUMap) {

    if (pUMap == NULL || pUMap->capacity == 0) {
        return 1.0F;
    }
    return (float)pUMap->size / (float)pUMap->capacity;
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

    // If slot is occupied and it's the same key, update the value
    if (pUMap->pEntries[index].occupied) {
        if (pUMap->pEntries[index].hash == hash &&
            pUMap->equalFunc (pUMap->pEntries[index].pKey, pKey, pUMap->keySize)) {
            // Update existing value
            memcpy (pUMap->pEntries[index].pValue, pValue, pUMap->valueSize);
            return true;
        }
        // Table is full (no empty slots found)
        return false;
    }

    // Insert new key-value pair
    memcpy (pUMap->pEntries[index].pKey, pKey, pUMap->keySize);
    memcpy (pUMap->pEntries[index].pValue, pValue, pUMap->valueSize);
    pUMap->pEntries[index].hash     = hash;
    pUMap->pEntries[index].occupied = true;
    pUMap->size++;

    return true;
}

bool UMap_Find (UMap_t const* pUMap, void const* pKey, void* pOutValue) {

    if (pUMap == NULL || pKey == NULL || pOutValue == NULL) {
        return false;
    }

    uint32_t hash  = pUMap->hashFunc (pKey, pUMap->keySize);
    uint16_t index = UMap_FindIndex (pUMap, pKey, hash);

    if (pUMap->pEntries[index].occupied && pUMap->pEntries[index].hash == hash &&
        pUMap->equalFunc (pUMap->pEntries[index].pKey, pKey, pUMap->keySize)) {
        memcpy (pOutValue, pUMap->pEntries[index].pValue, pUMap->valueSize);
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
        pUMap->equalFunc (pUMap->pEntries[index].pKey, pKey, pUMap->keySize)) {
        return pUMap->pEntries[index].pValue;
    }

    return NULL;
}

bool UMap_Contains (UMap_t const* pUMap, void const* pKey) {

    if (pUMap == NULL || pKey == NULL) {
        return false;
    }

    uint32_t hash  = pUMap->hashFunc (pKey, pUMap->keySize);
    uint16_t index = UMap_FindIndex (pUMap, pKey, hash);

    return pUMap->pEntries[index].occupied && pUMap->pEntries[index].hash == hash &&
           pUMap->equalFunc (pUMap->pEntries[index].pKey, pKey, pUMap->keySize);
}

void UMap_Clear (UMap_t* pUMap) {

    if (pUMap == NULL) {
        return;
    }

    for (uint16_t i = 0; i < pUMap->capacity; ++i) {
        pUMap->pEntries[i].occupied = false;
        pUMap->pEntries[i].hash     = 0;
    }

    pUMap->size = 0;
}