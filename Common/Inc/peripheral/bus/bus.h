#ifndef PERIPHERAL_BUS_BUS_H
#define PERIPHERAL_BUS_BUS_H

#include "common.h"
#include "conf/board.h"
#include "conf/ids.h"
#include "peripheral/bus/i2c.h"
#include "peripheral/bus/spi.h"
#include "peripheral/bus/uart.h"
#include <stdbool.h>
#include <stdint.h>



typedef eSTATUS_t (*BusReadFn_t) (eBUS_ID_t busId, eDEVICE_ID_t deviceId, uint8_t* pData, size_t size);
typedef eSTATUS_t (*BusWriteFn_t) (eBUS_ID_t busId, eDEVICE_ID_t deviceId, uint8_t const* pData, size_t size);
typedef eSTATUS_t (*BusWriteReadFn_t) (
eBUS_ID_t busId,
eDEVICE_ID_t deviceId,
uint8_t const* pTxData,
uint8_t* pRxData,
size_t size
);

typedef struct {
    BusReadFn_t read;
    BusWriteFn_t write;
    BusWriteReadFn_t writeRead;
} BusInterface_t;

typedef struct {
    DeviceBoardConf_t deviceBoardConf;
    BusBoardConf_t busBoardConf;
} BusInitConf_t;


eSTATUS_t BusInit (BusInitConf_t conf, BusInterface_t* pOutBusInterface);

#define BUS_INIT(pSTATUS, DEV_BOARD_CONF, BUS_BOARD_CONF, pOUT_BUS_INTERFACE) \
    do {                                                                      \
        BusInitConf_t conf   = { 0 };                                         \
        conf.deviceBoardConf = (DEV_BOARD_CONF);                              \
        conf.busBoardConf    = (BUS_BOARD_CONF);                              \
        *(pSTATUS)           = BusInit (conf, pOUT_BUS_INTERFACE);            \
    } while (0)


#endif // PERIPHERAL_BUS_BUS_H