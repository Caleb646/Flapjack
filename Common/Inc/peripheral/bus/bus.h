#ifndef PERIPHERAL_BUS_BUS_H
#define PERIPHERAL_BUS_BUS_H

#include "common.h"
#include "conf/board.h"
#include "conf/ids.h"
#include "peripheral/bus/bus_core.h"
#include "peripheral/bus/i2c.h"
#include "peripheral/bus/spi.h"
#include "peripheral/bus/uart.h"
#include <stdbool.h>
#include <stdint.h>

// clang-format off
typedef eSTATUS_t (*BusReadFn_t) (void* pCtx, eDEVICE_ID_t deviceId, uint8_t* pData, size_t size);
typedef eSTATUS_t (*BusWriteFn_t) (void* pCtx, eDEVICE_ID_t deviceId, uint8_t const* pData, size_t size);
typedef eSTATUS_t (*BusWriteReadFn_t) (void* pCtx, eDEVICE_ID_t deviceId, uint8_t const* pTxData, uint8_t* pRxData, size_t size);
typedef eSTATUS_t (*BusTransactionsFn_t) (void* pCtx, eDEVICE_ID_t deviceId, BusTransaction_t* pTransactions, size_t nTransactions);

typedef struct {
    BusReadFn_t read;
    BusWriteFn_t write;
    BusWriteReadFn_t writeRead;
    BusTransactionsFn_t transactions;
    eDEVICE_ID_t deviceId;
    void* pCtx;
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

#define BUS_READ(BUS_INTERFACE, pDATA, SIZE) \
    (BUS_INTERFACE).read ((BUS_INTERFACE).pCtx, (BUS_INTERFACE).deviceId, (pDATA), (SIZE))

#define BUS_WRITE(BUS_INTERFACE, pDATA, SIZE) \
    (BUS_INTERFACE).write ((BUS_INTERFACE).pCtx, (BUS_INTERFACE).deviceId, (pDATA), (SIZE))

#define BUS_WRITE_READ(BUS_INTERFACE, pTX_DATA, pRX_DATA, SIZE) \
    (BUS_INTERFACE).writeRead ((BUS_INTERFACE).pCtx, (BUS_INTERFACE).deviceId, (pTX_DATA), (pRX_DATA), (SIZE))

// clang-format on

#define BUS_TRANSACTIONS_2WRITES(pSTATUS, BUS_INTERFACE, pTX1, TX1_SIZE, pTX2, TX2_SIZE)               \
    do {                                                                                               \
        BusTransaction_t trans[2U] = { 0 };                                                            \
        trans[0].pTxData           = (pTX1);                                                           \
        trans[0].txSize            = (TX1_SIZE);                                                       \
        trans[0].pRxData           = NULL;                                                             \
        trans[0].rxSize            = 0;                                                                \
        trans[1].pTxData           = (pTX2);                                                           \
        trans[1].txSize            = (TX2_SIZE);                                                       \
        trans[1].pRxData           = NULL;                                                             \
        trans[1].rxSize            = 0;                                                                \
        if ((BUS_INTERFACE).transactions == NULL) {                                                    \
            *(pSTATUS) = eSTATUS_FAILURE;                                                              \
            break;                                                                                     \
        }                                                                                              \
        *(pSTATUS) =                                                                                   \
        (BUS_INTERFACE).transactions ((BUS_INTERFACE).pCtx, (BUS_INTERFACE).deviceId, &trans[0U], 2U); \
    } while (0)


#endif // PERIPHERAL_BUS_BUS_H