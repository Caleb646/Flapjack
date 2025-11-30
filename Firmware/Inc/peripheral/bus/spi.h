#ifndef PERIPHS_SPI_H
#define PERIPHS_SPI_H

#include "conf/board.h"
#include "conf/conf.h"
#include "core/core.h"
#include "hal.h"
#include "peripheral/bus/common.h"
#include "peripheral/gpio.h"
#include <stdint.h>
#include <string.h>

typedef struct {
    BusDesc_t* pBusDesc;
} SPIInitConf_t;

typedef struct {
    vIO_t* pNss;
    eDEVICE_ID_t deviceId;
} SPIActiveOperation_t;

typedef struct {
    eBUS_ID_t busId;
    // eDEVICE_ID_t deviceId;
    SPI_HandleTypeDef handle;
    struct {
        vIO_t* gpios[SPI_MAX_DEVICES_PER_BUS];
        eDEVICE_ID_t ids[SPI_MAX_DEVICES_PER_BUS];
    } connectedDevices;
    uint8_t nDevices;
    SPIActiveOperation_t activeOperation;
    bool isInitialized;
} SPIBus_t;

// typedef SPIBus_t volatile vSPIBus_t;
typedef SPIBus_t vSPIBus_t;

// clang-format off
vSPIBus_t* SPIGetBusById (eBUS_ID_t busId);
eSTATUS_t SPIInit (SPIInitConf_t conf, vSPIBus_t* pOutBus);
eSTATUS_t SPIRead_Blocking (vSPIBus_t* pBus, eDEVICE_ID_t deviceId, uint8_t* pData, size_t size);
eSTATUS_t SPIWrite_Blocking (vSPIBus_t* pBus, eDEVICE_ID_t deviceId, uint8_t const* pData, size_t size);
eSTATUS_t SPIWriteRead_Blocking (vSPIBus_t* pBus, eDEVICE_ID_t deviceId, uint8_t const* pTxData, uint8_t* pRxData, size_t size);
eSTATUS_t SPITransactions_Blocking(vSPIBus_t* pBus, eDEVICE_ID_t deviceId, BusTransaction_t* pTransactions, size_t nTransactions);
// clang-format on

#define SPI_INIT(pBUS_DESC) SPIInit ((SPIInitConf_t){ .pBusDesc = (pBUS_DESC) }, NULL)

#endif /* PERIPHS_SPI_H */