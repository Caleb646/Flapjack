#ifndef PERIPHS_SPI_H
#define PERIPHS_SPI_H

#include "common.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "hal.h"
#include "peripheral/gpio.h"
#include <stdint.h>
#include <string.h>

#define SPI_MAX_DEVICES_PER_BUS 3U

typedef struct {
    eDEVICE_ID_t deviceId;
    eGPIO_ID_t nssId;
} SPIInitConf_t;

typedef struct {
    eBUS_ID_t busId;
    eDEVICE_ID_t deviceId;
    SPI_HandleTypeDef handle;
    eGPIO_ID_t nss[SPI_MAX_DEVICES_PER_BUS];
    eDEVICE_ID_t deviceIds[SPI_MAX_DEVICES_PER_BUS];
    uint8_t nDevices;
    bool isInitialized;
    bool useDMA;
} SPIBus_t;

// typedef SPIBus_t volatile vSPIBus_t;
typedef SPIBus_t vSPIBus_t;

eSTATUS_t SPIInit (SPIInitConf_t conf, SPIBoardConf_t boardConf);
eSTATUS_t
SPIRead_Blocking (eBUS_ID_t busId, eDEVICE_ID_t deviceId, uint8_t* pData, uint16_t size);
eSTATUS_t
SPIWrite_Blocking (eBUS_ID_t busId, eDEVICE_ID_t deviceId, uint8_t const* pData, uint16_t size);
eSTATUS_t
SPIWriteRead_Blocking (eBUS_ID_t busId, eDEVICE_ID_t deviceId, uint8_t const* pTxData, uint8_t* pRxData, uint16_t size);

// eSTATUS_t SPIRead (eBUS_ID_t busId, eDEVICE_ID_t deviceId, uint8_t* pData, uint16_t size);
// eSTATUS_t SPIWrite (eBUS_ID_t busId, eDEVICE_ID_t deviceId, uint8_t const* pData, uint16_t size);
// eSTATUS_t
// SPIWriteRead (eBUS_ID_t busId, eDEVICE_ID_t deviceId, uint8_t const* pTxData, uint8_t* pRxData, uint16_t size);

#define SPI_INIT_FROM_BOARD_CONF(pSTATUS, DEVICE_BOARD_CONF, BOARD_CONF) \
    do {                                                                 \
        SPIInitConf_t conf = { 0 };                                      \
        conf.deviceId      = (DEVICE_BOARD_CONF).deviceId;               \
        conf.nssId         = (DEVICE_BOARD_CONF).nssId;                  \
        *(pSTATUS)         = SPIInit (conf, (BOARD_CONF));               \
    } while (0)


#endif /* PERIPHS_SPI_H */