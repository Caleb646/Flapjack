#ifndef DEVICE_FLASH_FLASH_H
#define DEVICE_FLASH_FLASH_H

#include "common.h"
#include "peripheral/bus/bus.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    DeviceBoardConf_t boardConf;
} FlashInitConf_t;

typedef struct {
    eBUS_ID_t busId;
    eDEVICE_ID_t deviceId;
    BusInterface_t bus;
    bool isInitialized;
} Flash_t;

typedef Flash_t vFlash_t;

// clang-format off

eSTATUS_t FlashInit (FlashInitConf_t conf, Flash_t* pOutFlash);
eSTATUS_t FlashWrite (vFlash_t* pFlash, uint32_t addr, uint8_t const* pData, uint32_t size, uint32_t* pBytesWritten);

// clang-format on


#endif // DEVICE_FLASH_FLASH_H