#include "peripheral/bus/bus.h"
#include "conf/board.h"
#include "conf/ids.h"
#include "core/core.h"
#include "peripheral/bus/i2c.h"
#include "peripheral/bus/spi.h"
#include "peripheral/bus/uart.h"
#include <stdbool.h>
#include <stdint.h>


eSTATUS_t BusInit (BusInitConf_t conf, BusVTable_t* pOutBusVTable) {

    if (pOutBusVTable == NULL) {
        return eSTATUS_FAILURE;
    }

    eSTATUS_t status = eSTATUS_SUCCESS;

    DeviceBoardConf_t device = conf.deviceBoardConf;
    eDEVICE_ID_t deviceId    = device.deviceId;
    // TODO: currently not used
    // bool usingDMAForBus        = device.useDMAForWrites;
    // bool usingInterruptsForBus = device.useInterruptsForWrites;
    BusBoardConf_t bus = conf.busBoardConf;
    eBUS_ID_t busId    = bus.busId;

    if (BUS_ID_IS_I2C (busId)) {
        return eSTATUS_FAILURE;
    }

    if (BUS_ID_IS_SPI (busId)) {

        status = SPI_INIT (device, bus);
        if (FJ_FAIL (status)) {
            return eSTATUS_FAILURE;
        }

        pOutBusVTable->ReadBlocking = SPI_READ_BLOCKING;
        pOutBusVTable->ReadIT       = NULL;

        pOutBusVTable->WriteBlocking = SPI_WRITE_BLOCKING;

        pOutBusVTable->WriteReadBlocking = SPI_WRITE_READ_BLOCKING;

        pOutBusVTable->TransactionsBlocking = SPI_TRANSACTIONS_BLOCKING;

        pOutBusVTable->RegisterCallback = NULL;

        pOutBusVTable->deviceId = deviceId;
        pOutBusVTable->pCtx     = SPIGetBusById (busId);
        return eSTATUS_SUCCESS;
    }

    if (BUS_ID_IS_UART (busId)) {

        status = UART_INIT (device, bus);
        if (FJ_FAIL (status)) {
            return eSTATUS_FAILURE;
        }

        pOutBusVTable->ReadBlocking = UART_READ_BLOCKING;
        pOutBusVTable->ReadIT       = UART_READ_IT;

        pOutBusVTable->WriteBlocking = UART_WRITE_BLOCKING;

        pOutBusVTable->WriteReadBlocking = NULL;

        pOutBusVTable->TransactionsBlocking = NULL;

        pOutBusVTable->RegisterCallback = UART_REGISTER_CALLBACK;

        pOutBusVTable->deviceId = deviceId;
        pOutBusVTable->pCtx     = UARTGetBusById (busId);
        return eSTATUS_SUCCESS;
    }
    // invalid bus type
    return eSTATUS_FAILURE;
}