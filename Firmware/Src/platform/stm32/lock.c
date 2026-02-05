#include <stdint.h>

#include "common.h"

#include "drivers/core/lock.h"

#include "platform/platform.h"

#define HSEM_MAX_IDS 32

static uint8_t s_hsemId TARG_SHARED_MEM_DATA_SECTION = 1U;

void Plat_SpinLock_Init (SpinLock_t* pLock) {

    while (HAL_HSEM_FastTake (0) != HAL_OK) {
        // wait
    }

    pLock->semId = s_hsemId++;
    FJ_ASSERT (pLock->semId < HSEM_MAX_IDS);

    HAL_HSEM_Release (0, 0);
}

void Plat_SpinLock_Take (SpinLock_t* pLock) {

    while (HAL_HSEM_FastTake (pLock->semId) != HAL_OK) {
        // wait
    }
}

void Plat_SpinLock_Release (SpinLock_t* pLock) {

    HAL_HSEM_Release (pLock->semId, 0);
}