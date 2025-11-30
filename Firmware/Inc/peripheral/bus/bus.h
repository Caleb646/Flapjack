#ifndef PERIPHERAL_BUS_BUS_H
#define PERIPHERAL_BUS_BUS_H

#include "conf/board.h"
#include "conf/ids.h"
#include "core/core.h"
#include "peripheral/bus/common.h"
#include "peripheral/bus/i2c.h"
#include "peripheral/bus/spi.h"
#include "peripheral/bus/uart.h"
#include <stdbool.h>
#include <stdint.h>

typedef uint32_t eBUS_OP_MODE_t;
enum {
    eBUS_OP_MODE_NULL  = 0U,
    eBUS_OP_MODE_BLOCK = 1U,
    eBUS_OP_MODE_IT    = 2U,
    eBUS_OP_MODE_DMA   = 3U
};

typedef uint32_t eBUS_OP_DIR_t;
enum {
    eBUS_OP_DIR_NULL        = 0U,
    eBUS_OP_DIR_READ        = (1U << 0U),
    eBUS_OP_DIR_WRITE       = (1U << 4U),
    eBUS_OP_DIR_WRITE_READ  = (1U << 8U),
    eBUS_OP_DIR_TRANSACTION = (1U << 12U)
};

typedef uint32_t eBUS_TYPE_t;
enum {
    eBUS_TYPE_NULL = 0U,
    eBUS_TYPE_I2C  = (1 << 2U),
    eBUS_TYPE_SPI  = (1 << 3U),
    eBUS_TYPE_UART = (1 << 4U)
};


#define BUS_MAKE_TYPE_ID(TYPE, MODE) ((uint32_t)(TYPE) | (uint32_t)(MODE))
#define BUS_MAKE_OP_ID(DIR, MODE)    ((uint32_t)(DIR) << (uint32_t)(MODE))
#define BUS_SUPPORTS_OP(pBUS, DIR, MODE) \
    (((pBUS)->supportedOps & BUS_MAKE_OP_ID ((DIR), (MODE))) != 0U)

typedef struct {
    // BusReadFn_t ReadBlocking;
    // BusReadFn_t ReadIT;
    // BusWriteFn_t WriteBlocking;
    // BusWriteReadFn_t WriteReadBlocking;
    // BusTransactionsFn_t TransactionsBlocking;
    // BusRegisterCallbackFn_t RegisterCallback;
    eDEVICE_ID_t deviceId;
    eBUS_ID_t busId;
    eBUS_TYPE_t busType;
    uint32_t supportedOps; // eBUS_OP_MODE_t | eBUS_OP_DIR_t
    void* pCtx;            // spi, i2c, uart bus
    bool isInitialized;
} Bus_t;

#define BUS_IS_SPI(pBUS)  (BUS_ID_IS_SPI ((pBUS)->busId))
#define BUS_IS_UART(pBUS) (BUS_ID_IS_UART ((pBUS)->busId))
#define BUS_VALID(pBUS)   ((pBUS) != NULL && (pBUS)->isInitialized && (pBUS)->pCtx != NULL)

typedef struct {
    DevDesc_t* pDevDesc;
} BusInitConf_t;

eSTATUS_t Bus_Init (BusInitConf_t conf, Bus_t* pOutBus);
eSTATUS_t Bus_Read (Bus_t* pBus, eBUS_OP_MODE_t opMode, uint8_t* pData, size_t size);
eSTATUS_t Bus_Write (Bus_t* pBus, eBUS_OP_MODE_t opMode, uint8_t const* pData, size_t size);
eSTATUS_t Bus_WriteRead (Bus_t* pBus, eBUS_OP_MODE_t opMode, uint8_t const* pTxData, uint8_t* pRxData, size_t size);
eSTATUS_t Bus_Transactions (Bus_t* pBus, eBUS_OP_MODE_t opMode, BusTransaction_t* pTransactions, size_t nTransactions);
eSTATUS_t Bus_RegisterCallback (Bus_t* pBus, BusCallback_t callback);

#define BUS_INIT(pDEV_DESC, pOUT_BUS_VTABLE) \
    Bus_Init ((BusInitConf_t){ .pDevDesc = (pDEV_DESC) }, pOUT_BUS_VTABLE)

#define BUS_READ_BLOCK(pBUS, pDATA, SIZE)  Bus_Read ((pBUS), eBUS_OP_MODE_BLOCK, (pDATA), (SIZE))
#define BUS_READ_IT(pBUS, pDATA, SIZE)     Bus_Read ((pBUS), eBUS_OP_MODE_IT, (pDATA), (SIZE))

#define BUS_WRITE_BLOCK(pBUS, pDATA, SIZE) Bus_Write ((pBUS), eBUS_OP_MODE_BLOCK, (pDATA), (SIZE))
#define BUS_WRITE_READ_BLOCK(pBUS, pTX_DATA, pRX_DATA, SIZE) \
    Bus_WriteRead ((pBUS), eBUS_OP_MODE_BLOCK, (pTX_DATA), (pRX_DATA), (SIZE))

#define BUS_CREATE_TRANSACTIONS(NAME, NUMBER_OF_TRANSACTIONS) \
    BusTransaction_t NAME[NUMBER_OF_TRANSACTIONS]

#define BUS_CREATE_TRANSACTION(pTX, TX_SIZE, pRX, RX_SIZE) \
    { .pTxData = (pTX), .txSize = (TX_SIZE), .pRxData = (pRX), .rxSize = (RX_SIZE) }

#define BUS_CREATE_WRITE_TRANSACTION(pTX, TX_SIZE) \
    { .pTxData = (pTX), .txSize = (TX_SIZE), .pRxData = NULL, .rxSize = 0U }

#define BUS_DO_TRANSACTIONS_BLOCK(pBUS, pTRANSACTIONS, N_TRANSACTIONS) \
    Bus_Transactions ((pBUS), eBUS_OP_MODE_BLOCK, (pTRANSACTIONS), (N_TRANSACTIONS))

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