#include "peripheral/bus/bus.h"
#include "common.h"
#include "conf/board.h"
#include "conf/ids.h"
#include "peripheral/bus/i2c.h"
#include "peripheral/bus/spi.h"
#include "peripheral/bus/uart.h"
#include <stdbool.h>
#include <stdint.h>


eSTATUS_t BusInit (BusInitConf_t conf, BusInterface_t* pOutBusInterface) {

    if (pOutBusInterface == NULL) {
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

        SPI_INIT (&status, device, bus);
        if (STATUS_FAIL (status)) {
            return eSTATUS_FAILURE;
        }

        pOutBusInterface->read      = SPI_READ_BLOCKING;
        pOutBusInterface->write     = SPI_WRITE_BLOCKING;
        pOutBusInterface->writeRead = SPI_WRITE_READ_BLOCKING;
        pOutBusInterface->deviceId  = deviceId;
        pOutBusInterface->pCtx      = SPIGetBusById (busId);
        return eSTATUS_SUCCESS;
    }

    if (BUS_ID_IS_UART (busId)) {

        UART_INIT (&status, device, bus);
        if (STATUS_FAIL (status)) {
            return eSTATUS_FAILURE;
        }

        pOutBusInterface->read      = UART_READ_BLOCKING;
        pOutBusInterface->write     = UART_WRITE_BLOCKING;
        pOutBusInterface->writeRead = NULL;
        pOutBusInterface->deviceId  = deviceId;
        pOutBusInterface->pCtx      = UARTGetBusById (busId);
        return eSTATUS_SUCCESS;
    }
    // invalid bus type
    return eSTATUS_FAILURE;
}