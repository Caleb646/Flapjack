#include "core/core.h"

#define SHARED_MEM_SIZE 2048U
FJ_DEFINE_SHARED(uint8_t, s_SharedMemory[SHARED_MEM_SIZE]);
FJ_DEFINE_SHARED(uint32_t, s_SharedMemoryAllocated) = 0;

eSTATUS_t Core_Init (void) {

    if (CoreShared_Init () != eSTATUS_SUCCESS) {
        return eSTATUS_FAILURE;
    }

    if (SyncInit () != eSTATUS_SUCCESS) {
        return eSTATUS_FAILURE;
    }

    if (LoggerInit () != eSTATUS_SUCCESS) {
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

void* Allocate(uint32_t size) {

    if(!size || (s_SharedMemoryAllocated + size) > SHARED_MEM_SIZE) {
        return NULL;
    }

    void* ptr = &s_SharedMemory[s_SharedMemoryAllocated];
    s_SharedMemoryAllocated += size;
    return ptr;
}