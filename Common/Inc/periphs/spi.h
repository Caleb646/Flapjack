#ifndef PERIPHS_SPI_H
#define PERIPHS_SPI_H

#include "common.h"
#include "hal.h"
#include "periphs/gpio.h"
#include <stdint.h>

#define SPI_MAX_DEVICES_PER_BUS 3U

typedef uint8_t eSPI_BUS_ID_t;
enum {
    eSPI_1_BUS_ID = 0,
    eSPI_2_BUS_ID,
    eSPI_3_BUS_ID,
    eSPI_4_BUS_ID,
    eSPI_5_BUS_ID,
    eSPI_BUS_ID_MAX
};

typedef uint32_t eSPI_SPEED_t;
enum {
    eSPI_SPEED_500_KHZ = 500000U,  // 500 KHz
    eSPI_SPEED_1_MHZ   = 1000000U, // 1 MHz
    eSPI_SPEED_8_MHZ   = 8000000U, // 8 MHz
    eSPI_SPEED_20_MHZ  = 20000000U // 20 MHz
};

typedef struct {
    eSPI_BUS_ID_t busId;
    eSPI_SPEED_t speed;
    eDEVICE_ID_t deviceId;
} SPIInitConf_t;

typedef struct {
    SPI_HandleTypeDef handle;
    eSPI_BUS_ID_t busId;
    eSPI_SPEED_t speed;
    BOOL_t isInitialized;
    GPIOSPI_t gpio[SPI_MAX_DEVICES_PER_BUS];
    eDEVICE_ID_t deviceIds[SPI_MAX_DEVICES_PER_BUS];
    uint32_t nDevices;
} SPIBus_t;

eSTATUS_t SPIInit (SPIInitConf_t const* pConf);
eSTATUS_t SPIRead_Blocking (eSPI_BUS_ID_t busId, uint8_t* pData, uint16_t size);
eSTATUS_t SPIWrite_Blocking (eSPI_BUS_ID_t busId, uint8_t const* pData, uint16_t size);
eSTATUS_t
SPIWriteRead_Blocking (eSPI_BUS_ID_t busId, uint8_t const* pTxData, uint8_t* pRxData, uint16_t size);


#endif /* PERIPHS_SPI_H */