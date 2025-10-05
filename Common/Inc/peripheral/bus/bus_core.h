#ifndef PERIPHERAL_BUS_BUS_CORE_H
#define PERIPHERAL_BUS_BUS_CORE_H

#include <stdint.h>
#include <string.h>

typedef uint8_t eBUS_CALLBACK_ID_t;
enum {
    eBUS_CALLBACK_ID_RX = 0, // data register NOT empty
    eBUS_CALLBACK_ID_RX_COMPLETE,
    eBUS_CALLBACK_ID_RX_FIFO_FULL,
    eBUS_CALLBACK_ID_TX, // data register empty
    eBUS_CALLBACK_ID_TX_COMPLETE,
    eBUS_CALLBACK_ID_TX_FIFO_EMPTY,
    eBUS_CALLBACK_ID_ERROR,
    eBUS_NUMBER_OF_CALLBACK_IDS
};

#define BUS_CALLBACK_ID_VALID(ID) ((ID) < eBUS_NUMBER_OF_CALLBACK_IDS)

// clang-format off

typedef struct BusCallbackData_s {
    eBUS_CALLBACK_ID_t cbId;
    void* pUserCtx;
} BusCallbackData_t;

typedef void (*BusCallbackFn_t) (BusCallbackData_t data);

typedef struct {
    BusCallbackData_t data;
    BusCallbackFn_t Callback;
} BusCallback_t;

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