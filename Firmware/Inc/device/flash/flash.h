#ifndef DEVICE_FLASH_FLASH_H
#define DEVICE_FLASH_FLASH_H

#include "core/core.h"
#include "peripheral/bus/bus.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    DevDesc_t* pDevDesc;
    Bus_t* pBus;
} FlashInitConf_t;

typedef struct {
    eDEVICE_ID_t deviceId;
    Bus_t bus;
    bool isInitialized;
} Flash_t;

typedef Flash_t vFlash_t;

// clang-format off

eSTATUS_t FlashInit (FlashInitConf_t conf, Flash_t* pOutFlash);
eSTATUS_t FlashStart (vFlash_t* pFlash);
eSTATUS_t FlashWrite (vFlash_t* pFlash, uint32_t addr, uint8_t const* pData, uint32_t size, uint32_t* pBytesWritten);

vFlash_t const* FlashGetActiveDevice(void);
vFlash_t* FlashGetMutableActiveDevice(void);

// clang-format on


#endif // DEVICE_FLASH_FLASH_H