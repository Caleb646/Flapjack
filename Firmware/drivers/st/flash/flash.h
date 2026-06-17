#ifndef DEVICE_FLASH_FLASH_H
#define DEVICE_FLASH_FLASH_H

#include "core/core.h"

#include "drivers/bus/spi.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    SpiDev_t spiDev;
} Flash_t;

// clang-format off

FJ_DECLARE_SHARED (Flash_t, g_Flash);

eSTATUS_t Flash_Init_ (Flash_t* pOutFlash);
static inline eSTATUS_t Flash_Init(void) {
    return Flash_Init_(&g_Flash);
}

uint32_t Flash_Write (Flash_t* pFlash, uint32_t addr, uint8_t const* pData, uint32_t size);

// clang-format on


#endif // DEVICE_FLASH_FLASH_H