#include "core/stl/vector.h"
#include "core/core.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

eSTATUS_t Vector_Init (Vector_t* pVector, void* pBuffer, uint16_t capacity, uint16_t elementSize, bool isShared) {

    if (pVector == NULL || pBuffer == NULL || elementSize == 0) {
        return eSTATUS_FAILURE;
    }

    pVector->processID   = isShared ? 1 : 0; // Set processID to 1 for shared, 0 for non-shared
    pVector->pData       = pBuffer;
    pVector->capacity    = capacity;
    pVector->elementSize = elementSize;
    pVector->size        = 0;

    return eSTATUS_SUCCESS;
}

bool Vector_IsEmpty (Vector_t const* pVector) {

    if (pVector == NULL) {
        return true;
    }
    return pVector->size == 0 ? true : false;
}

bool Vector_IsFull (Vector_t const* pVector) {

    if (pVector == NULL) {
        return true;
    }
    return pVector->size >= pVector->capacity ? true : false;
}

uint16_t Vector_Size (Vector_t const* pVector) {

    if (pVector == NULL) {
        return 0;
    }
    return pVector->size;
}

uint16_t Vector_Capacity (Vector_t const* pVector) {

    if (pVector == NULL) {
        return 0;
    }
    return pVector->capacity;
}

eSTATUS_t Vector_PushBack (Vector_t* pVector, void const* pElement) {

    if (pVector == NULL || pElement == NULL) {
        return eSTATUS_FAILURE;
    }

    if (pVector->size >= pVector->capacity) {
        return eSTATUS_FAILURE; // Vector is full
    }

    // Calculate the address where to insert the new element
    uint8_t* pDest = ((uint8_t*)pVector->pData) + (pVector->size * pVector->elementSize);

    // Copy the element to the vector
    memcpy (pDest, pElement, pVector->elementSize);

    pVector->size++;

    return eSTATUS_SUCCESS;
}

eSTATUS_t Vector_PopBack (Vector_t* pVector, void* pOutElement) {

    if (pVector == NULL) {
        return eSTATUS_FAILURE;
    }

    if (pVector->size == 0) {
        return eSTATUS_FAILURE; // Vector is empty
    }

    if (pOutElement != NULL) {
        // Calculate the address of the element to pop
        uint8_t* pSrc = ((uint8_t*)pVector->pData) + ((pVector->size - 1U) * pVector->elementSize);
        memcpy (pOutElement, pSrc, pVector->elementSize);
    }
    pVector->size--;

    return eSTATUS_SUCCESS;
}

void* Vector_At (Vector_t const* pVector, uint16_t index) {

    if (pVector == NULL) {
        return NULL;
    }

    if (index >= pVector->size) {
        return NULL; // Index out of bounds
    }

    // Calculate the address of the element at the given index
    return ((uint8_t*)pVector->pData) + (index * pVector->elementSize);
}

void* Vector_Front (Vector_t const* pVector) {

    if (pVector == NULL) {
        return NULL;
    }

    if (pVector->size == 0) {
        return NULL; // Vector is empty
    }

    return pVector->pData;
}

void* Vector_Back (Vector_t const* pVector) {

    if (pVector == NULL) {
        return NULL;
    }

    if (pVector->size == 0) {
        return NULL; // Vector is empty
    }

    // Calculate the address of the last element
    return ((uint8_t*)pVector->pData) + ((pVector->size - 1) * pVector->elementSize);
}

eSTATUS_t Vector_Insert (Vector_t* pVector, uint16_t index, void const* pElement) {

    if (pVector == NULL || pElement == NULL) {
        return eSTATUS_FAILURE;
    }

    if (pVector->size >= pVector->capacity || index > pVector->size) {
        return eSTATUS_FAILURE;
    }

    if (index < pVector->size) {
        // Move elements to make space for the new element
        uint8_t* pDest = ((uint8_t*)pVector->pData) + ((index + 1) * pVector->elementSize);
        uint8_t* pSrc  = ((uint8_t*)pVector->pData) + (index * pVector->elementSize);
        memmove (pDest, pSrc, (pVector->size - index) * pVector->elementSize);
    }

    // Insert the new element
    uint8_t* pInsertPos = ((uint8_t*)pVector->pData) + (index * pVector->elementSize);
    memcpy (pInsertPos, pElement, pVector->elementSize);

    pVector->size++;
    return eSTATUS_SUCCESS;
}

eSTATUS_t Vector_Erase (Vector_t* pVector, uint16_t index, void* pOutElement) {

    if (pVector == NULL) {
        return eSTATUS_FAILURE;
    }

    if (index >= pVector->size) {
        return eSTATUS_FAILURE; // Index out of bounds
    }

    // Copy the element to be erased if output pointer is provided
    if (pOutElement != NULL) {
        uint8_t* pSrc = ((uint8_t*)pVector->pData) + (index * pVector->elementSize);
        memcpy (pOutElement, pSrc, pVector->elementSize);
    }

    // Move elements to fill the gap
    if (index < pVector->size - 1) {
        uint8_t* pDest = ((uint8_t*)pVector->pData) + (index * pVector->elementSize);
        uint8_t* pSrc  = ((uint8_t*)pVector->pData) + ((index + 1) * pVector->elementSize);
        memmove (pDest, pSrc, (pVector->size - index - 1) * pVector->elementSize);
    }

    pVector->size--;
    return eSTATUS_SUCCESS;
}

void Vector_Clear (Vector_t* pVector) {

    if (pVector != NULL) {
        pVector->size = 0;
    }
}

eSTATUS_t Vector_Resize (Vector_t* pVector, uint16_t newSize, void const* pFillValue) {

    if (pVector == NULL) {
        return eSTATUS_FAILURE;
    }

    if (newSize > pVector->capacity) {
        return eSTATUS_FAILURE; // New size exceeds capacity
    }

    // If growing and fill value is provided, fill new elements
    if (newSize > pVector->size && pFillValue != NULL) {
        for (uint16_t i = pVector->size; i < newSize; i++) {
            uint8_t* pDest = ((uint8_t*)pVector->pData) + (i * pVector->elementSize);
            memcpy (pDest, pFillValue, pVector->elementSize);
        }
    }

    pVector->size = newSize;
    return eSTATUS_SUCCESS;
}

void* Vector_Data (Vector_t const* pVector) {

    if (pVector == NULL) {
        return NULL;
    }
    return pVector->pData;
}