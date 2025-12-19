#include "peripheral/bus/bus.h"
#include "conf/board.h"
#include "conf/ids.h"
#include "core/core.h"
#include "peripheral/bus/i2c.h"
#include "peripheral/bus/spi.h"
#include "peripheral/bus/uart.h"
#include <stdbool.h>
#include <stdint.h>

eSTATUS_t Bus_Init (BusInitConf_t conf, Bus_t* pOutBus) {

    if (FJ_IS_NULL (pOutBus) || FJ_IS_NULL (conf.pDevDesc) || !DEV_DESC_HAS_BUS (conf.pDevDesc)) {
        return eSTATUS_NULL_ARG;
    }

    eSTATUS_t status      = eSTATUS_OK;
    DevDesc_t* pDevDesc   = conf.pDevDesc;
    BusDesc_t* pBusDesc   = DEV_DESC_GET_BUS (pDevDesc);
    eDEVICE_ID_t deviceId = DEV_DESC_GET_ID (pDevDesc);
    eBUS_ID_t busId       = DEV_DESC_GET_BUS_ID (pDevDesc);

    memset (pOutBus, 0, sizeof (Bus_t));
    pOutBus->supportedOps = 0;
    pOutBus->deviceId     = deviceId;
    pOutBus->busId        = busId;
    pOutBus->pCtx         = NULL;

    if (BUS_ID_IS_I2C (busId)) {
        return eSTATUS_INVALID_ARG;
    }

    if (BUS_ID_IS_SPI (busId)) {

        status = SPI_INIT (pBusDesc);
        if (FJ_FAIL (status)) {
            return status;
        }

        pOutBus->busType = eBUS_TYPE_SPI;
        pOutBus->pCtx    = SPIGetBusById (busId);
        pOutBus->supportedOps |= BUS_MAKE_OP_ID (eBUS_OP_DIR_READ, eBUS_OP_MODE_BLOCK);
        pOutBus->supportedOps |= BUS_MAKE_OP_ID (eBUS_OP_DIR_WRITE, eBUS_OP_MODE_BLOCK);
        pOutBus->supportedOps |= BUS_MAKE_OP_ID (eBUS_OP_DIR_WRITE_READ, eBUS_OP_MODE_BLOCK);
        pOutBus->supportedOps |= BUS_MAKE_OP_ID (eBUS_OP_DIR_TRANSACTION, eBUS_OP_MODE_BLOCK);
    }

    if (BUS_ID_IS_UART (busId)) {

        status = UART_INIT (pBusDesc);
        if (FJ_FAIL (status)) {
            return eSTATUS_FAIL;
        }

        pOutBus->busType = eBUS_TYPE_UART;
        pOutBus->pCtx    = UARTGetBusById (busId);
        pOutBus->supportedOps |= BUS_MAKE_OP_ID (eBUS_OP_DIR_READ, eBUS_OP_MODE_BLOCK);
        pOutBus->supportedOps |= BUS_MAKE_OP_ID (eBUS_OP_DIR_WRITE, eBUS_OP_MODE_BLOCK);
        pOutBus->supportedOps |= BUS_MAKE_OP_ID (eBUS_OP_DIR_WRITE_READ, eBUS_OP_MODE_BLOCK);
        pOutBus->supportedOps |= BUS_MAKE_OP_ID (eBUS_OP_DIR_TRANSACTION, eBUS_OP_MODE_BLOCK);

        return eSTATUS_OK;
    } else {
        // invalid bus type
        return eSTATUS_UNSUPPORTED;
    }

    pOutBus->isInitialized = true;
    return eSTATUS_OK;
}

eSTATUS_t Bus_Read (Bus_t* pBus, eBUS_OP_MODE_t opMode, uint8_t* pData, size_t size) {

    if (!BUS_VALID (pBus) || FJ_IS_NULL (pData) || size == 0) {
        return eSTATUS_INVALID_ARG;
    }
    switch (BUS_MAKE_TYPE_ID (pBus->busType, opMode)) {
    case BUS_MAKE_TYPE_ID (eBUS_TYPE_SPI, eBUS_OP_MODE_BLOCK):
        return SPIRead_Blocking ((vSPIBus_t*)pBus->pCtx, pBus->deviceId, pData, size);
    case BUS_MAKE_TYPE_ID (eBUS_TYPE_UART, eBUS_OP_MODE_BLOCK):
        return UARTRead_Blocking ((vUARTBus_t*)pBus->pCtx, pBus->deviceId, pData, size);
    default: return eSTATUS_NOT_SUPPORTED;
    }
}

eSTATUS_t Bus_Write (Bus_t* pBus, eBUS_OP_MODE_t opMode, uint8_t const* pData, size_t size) {

    if (!BUS_VALID (pBus) || FJ_IS_NULL (pData) || size == 0) {
        return eSTATUS_INVALID_ARG;
    }
    switch (BUS_MAKE_TYPE_ID (pBus->busType, opMode)) {
    case BUS_MAKE_TYPE_ID (eBUS_TYPE_SPI, eBUS_OP_MODE_BLOCK):
        return SPIWrite_Blocking ((vSPIBus_t*)pBus->pCtx, pBus->deviceId, pData, size);
    case BUS_MAKE_TYPE_ID (eBUS_TYPE_UART, eBUS_OP_MODE_BLOCK):
        return UARTWrite_Blocking ((vUARTBus_t*)pBus->pCtx, pBus->deviceId, pData, size);
    default: return eSTATUS_NOT_SUPPORTED;
    }
}

eSTATUS_t Bus_WriteRead (Bus_t* pBus, eBUS_OP_MODE_t opMode, uint8_t const* pTxData, uint8_t* pRxData, size_t size) {

    if (!BUS_VALID (pBus) || FJ_IS_NULL (pTxData) || FJ_IS_NULL (pRxData) || size == 0) {
        return eSTATUS_INVALID_ARG;
    }
    switch (BUS_MAKE_TYPE_ID (pBus->busType, opMode)) {
    case BUS_MAKE_TYPE_ID (eBUS_TYPE_SPI, eBUS_OP_MODE_BLOCK):
        return SPIWriteRead_Blocking ((vSPIBus_t*)pBus->pCtx, pBus->deviceId, pTxData, pRxData, size);
    default: return eSTATUS_NOT_SUPPORTED;
    }
}

eSTATUS_t Bus_Transactions (Bus_t* pBus, eBUS_OP_MODE_t opMode, BusTransaction_t* pTransactions, size_t nTransactions) {

    if (!BUS_VALID (pBus) || FJ_IS_NULL (pTransactions) || nTransactions == 0) {
        return eSTATUS_INVALID_ARG;
    }
    switch (BUS_MAKE_TYPE_ID (pBus->busType, opMode)) {
    case BUS_MAKE_TYPE_ID (eBUS_TYPE_SPI, eBUS_OP_MODE_BLOCK):
        return SPITransactions_Blocking ((vSPIBus_t*)pBus->pCtx, pBus->deviceId, pTransactions, nTransactions);
    default: return eSTATUS_NOT_SUPPORTED;
    }
}

eSTATUS_t Bus_RegisterCallback (Bus_t* pBus, BusCallback_t callback) {

    if (!BUS_VALID (pBus) || FJ_IS_NULL (callback.Callback)) {
        return eSTATUS_INVALID_ARG;
    }

    switch (pBus->busType) {
    case eBUS_TYPE_UART:
        return UARTRegisterCallback ((vUARTBus_t*)pBus->pCtx, pBus->deviceId, callback);
    default: return eSTATUS_NOT_SUPPORTED;
    }
}