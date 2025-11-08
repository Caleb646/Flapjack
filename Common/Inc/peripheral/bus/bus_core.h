#ifndef PERIPHERAL_BUS_BUS_CORE_H
#define PERIPHERAL_BUS_BUS_CORE_H

#include "conf/conf.h"
#include "core/core.h"
#include <stdint.h>
#include <string.h>


typedef uint8_t eBUS_CALLBACK_ID_t;
enum {
    // if sub type is 0 then data register not empty interrupt
    eBUS_CALLBACK_ID_RX = 0,
    // if sub type is 0 then data register empty interrupt
    eBUS_CALLBACK_ID_TX,
    eBUS_CALLBACK_ID_ERROR,
    eBUS_NUMBER_OF_CALLBACK_IDS
};

typedef uint8_t eBUS_CALLBACK_SUB_ID_t;
enum {
    eBUS_CALLBACK_SUB_ID_RX_COMPLETE   = (1U << 0U),
    eBUS_CALLBACK_SUB_ID_RX_FIFO_FULL  = (1U << 1U),
    eBUS_CALLBACK_SUB_ID_TX_COMPLETE   = (1U << 2U),
    eBUS_CALLBACK_SUB_ID_TX_FIFO_EMPTY = (1U << 3U),
};

#define BUS_MAKE_SUB_CB_ID(...)   VALUES (OR_, __VA_ARGS__)
#define BUS_CALLBACK_ID_VALID(ID) ((ID) < eBUS_NUMBER_OF_CALLBACK_IDS)

// clang-format off

typedef struct BusCallbackData_s {
    eBUS_CALLBACK_ID_t cbId;
    eBUS_CALLBACK_SUB_ID_t cbActiveSubIds;
    eBUS_CALLBACK_SUB_ID_t currentlyActiveSubId;
    void* pUserCtx;
} BusCallbackData_t;

// a sub id of 0 means no sub id, just the main id so mark it is as valid
#define BUS_SUB_ID_IS_ACTIVE(CB_DATA, SUB_ID) (((CB_DATA).cbActiveSubIds & (SUB_ID)) != 0U || (SUB_ID) == 0U)
#define BUS_GET_ACTIVE_SUB_ID(CB_DATA) ((CB_DATA).currentlyActiveSubId)

typedef void (*BusCallbackFn_t) (BusCallbackData_t data);

typedef struct {
    BusCallbackData_t data;
    BusCallbackFn_t Callback;
} BusCallback_t;

#define BUS_DO_CALLBACK(pCALLBACK_STRUCT, ACTIVE_SUB_ID) \
    do {                                      \
        if ((pCALLBACK_STRUCT) != NULL && (pCALLBACK_STRUCT)->Callback != NULL) { \
            (pCALLBACK_STRUCT)->data.currentlyActiveSubId = (ACTIVE_SUB_ID); \
            (pCALLBACK_STRUCT)->Callback ((pCALLBACK_STRUCT)->data); \
        }                                     \
    } while (0)

typedef struct {
    uint8_t const* pTxData;
    uint8_t* pRxData;
    size_t txSize;
    size_t rxSize;
} BusTransaction_t;

typedef eSTATUS_t (*BusReadFn_t) (void* pCtx, eDEVICE_ID_t deviceId, uint8_t* pData, size_t size);
typedef eSTATUS_t (*BusWriteFn_t) (void* pCtx, eDEVICE_ID_t deviceId, uint8_t const* pData, size_t size);
typedef eSTATUS_t (*BusWriteReadFn_t) (void* pCtx, eDEVICE_ID_t deviceId, uint8_t const* pTxData, uint8_t* pRxData, size_t size);
typedef eSTATUS_t (*BusTransactionsFn_t) (void* pCtx, eDEVICE_ID_t deviceId, BusTransaction_t* pTransactions, size_t nTransactions);
typedef eSTATUS_t (*BusRegisterCallbackFn_t) (void* pCtx, eDEVICE_ID_t deviceId, BusCallback_t callback);

// clang-format on

#endif // PERIPHERAL_BUS_BUS_CORE_H