#ifndef PERIPHS_SPI_H
#define PERIPHS_SPI_H

#include "common.h"
#include "conf/conf.h"
#include "hal.h"
#include "periphs/gpio.h"
#include <stdint.h>

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
    BOOL_t isInitialized;
} SPIBus_t;

eSTATUS_t SPIInit (SPIInitConf_t conf, SPIBoardConf_t boardConf);
eSTATUS_t
SPIRead_Blocking (eBUS_ID_t busId, eDEVICE_ID_t deviceId, uint8_t* pData, uint16_t size);
eSTATUS_t
SPIWrite_Blocking (eBUS_ID_t busId, eDEVICE_ID_t deviceId, uint8_t const* pData, uint16_t size);
eSTATUS_t
SPIWriteRead_Blocking (eBUS_ID_t busId, eDEVICE_ID_t deviceId, uint8_t const* pTxData, uint8_t* pRxData, uint16_t size);

#define SPI_INIT(pSTATUS, BUS_ID, SPEED, DEVICE_ID, NSS_ID) \
    do {                                                    \
        SPIInitConf_t conf = { 0 };                         \
        conf.busId         = (BUS_ID);                      \
        conf.speed         = (SPEED);                       \
        conf.deviceId      = (DEVICE_ID);                   \
        conf.nssId         = (NSS_ID);                      \
        *(pSTATUS)         = SPIInit (conf);                \
    } while (0)


#endif /* PERIPHS_SPI_H */