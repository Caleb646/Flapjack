#ifndef PERIPHERAL_BUS_BUS_H
#define PERIPHERAL_BUS_BUS_H

#include "conf/board.h"
#include "conf/ids.h"
#include "core/core.h"
#include "peripheral/bus/bus_core.h"
#include "peripheral/bus/i2c.h"
#include "peripheral/bus/spi.h"
#include "peripheral/bus/uart.h"
#include <stdbool.h>
#include <stdint.h>


// clang-format off

#define BUS_VTABLE_VALID(pVTable) (pVTable != NULL && pVTable->pCtx != NULL)

typedef struct {
    BusReadFn_t ReadBlocking;
    BusReadFn_t ReadIT;
    BusWriteFn_t WriteBlocking;
    BusWriteReadFn_t WriteReadBlocking;
    BusTransactionsFn_t TransactionsBlocking;
    BusRegisterCallbackFn_t RegisterCallback;
    eDEVICE_ID_t deviceId;
    void* pCtx;
} BusVTable_t;

typedef struct {
    DeviceBoardConf_t deviceBoardConf;
    BusBoardConf_t busBoardConf;
} BusInitConf_t;

eSTATUS_t BusInit (BusInitConf_t conf, BusVTable_t* pOutBusVTable);

#define BUS_INIT(pSTATUS, DEV_BOARD_CONF, BUS_BOARD_CONF, pOUT_BUS_VTABLE) \
    do {                                                                      \
        BusInitConf_t conf   = { 0 };                                         \
        conf.deviceBoardConf = (DEV_BOARD_CONF);                              \
        conf.busBoardConf    = (BUS_BOARD_CONF);                              \
        *(pSTATUS)           = BusInit (conf, pOUT_BUS_VTABLE);              \
    } while (0)

#define BUS_READ_BLOCK(BUS_VTABLE, pDATA, SIZE) \
    (BUS_VTABLE).ReadBlocking ((BUS_VTABLE).pCtx, (BUS_VTABLE).deviceId, (pDATA), (SIZE))

#define BUS_READ_IT(BUS_VTABLE, pDATA, SIZE) \
    (BUS_VTABLE).ReadIT ((BUS_VTABLE).pCtx, (BUS_VTABLE).deviceId, (pDATA), (SIZE))

#define BUS_WRITE_BLOCK(BUS_VTABLE, pDATA, SIZE) \
    (BUS_VTABLE).WriteBlocking ((BUS_VTABLE).pCtx, (BUS_VTABLE).deviceId, (pDATA), (SIZE))

#define BUS_WRITE_READ_BLOCK(BUS_VTABLE, pTX_DATA, pRX_DATA, SIZE) \
    (BUS_VTABLE).WriteReadBlocking ((BUS_VTABLE).pCtx, (BUS_VTABLE).deviceId, (pTX_DATA), (pRX_DATA), (SIZE))

// clang-format on

#define BUS_TRANSACTIONS_2WRITES(pSTATUS, BUS_VTABLE, pTX1, TX1_SIZE, pTX2, TX2_SIZE)                 \
    do {                                                                                              \
        BusTransaction_t trans[2U] = { 0 };                                                           \
        trans[0].pTxData           = (pTX1);                                                          \
        trans[0].txSize            = (TX1_SIZE);                                                      \
        trans[0].pRxData           = NULL;                                                            \
        trans[0].rxSize            = 0;                                                               \
        trans[1].pTxData           = (pTX2);                                                          \
        trans[1].txSize            = (TX2_SIZE);                                                      \
        trans[1].pRxData           = NULL;                                                            \
        trans[1].rxSize            = 0;                                                               \
        if ((BUS_VTABLE).TransactionsBlocking == NULL) {                                              \
            *(pSTATUS) = eSTATUS_NULL_ARG;                                                            \
            break;                                                                                    \
        }                                                                                             \
        *(pSTATUS) =                                                                                  \
        (BUS_VTABLE).TransactionsBlocking ((BUS_VTABLE).pCtx, (BUS_VTABLE).deviceId, &trans[0U], 2U); \
    } while (0)

// Takes in a variable amount of sub callback ids
#define BUS_REG_CALLBACK(pSTATUS, BUS_VTABLE, pUSER_DATA, CALLBACK, CALLBACK_ID, ...)              \
    do {                                                                                           \
        if ((BUS_VTABLE).RegisterCallback == NULL) {                                               \
            *(pSTATUS) = eSTATUS_NULL_ARG;                                                         \
            break;                                                                                 \
        }                                                                                          \
        if (BUS_CALLBACK_ID_VALID (CALLBACK_ID) == false) {                                        \
            *(pSTATUS) = eSTATUS_INVALID_ARG;                                                      \
            break;                                                                                 \
        }                                                                                          \
        BusCallback_t cb       = { 0 };                                                            \
        cb.data.cbId           = (CALLBACK_ID);                                                    \
        cb.data.cbActiveSubIds = (BUS_MAKE_SUB_CB_ID (__VA_ARGS__));                               \
        cb.data.pUserCtx       = (pUSER_DATA);                                                     \
        cb.Callback            = (CALLBACK);                                                       \
        *(pSTATUS) = (BUS_VTABLE).RegisterCallback ((BUS_VTABLE).pCtx, (BUS_VTABLE).deviceId, cb); \
    } while (0)

#endif // PERIPHERAL_BUS_BUS_H