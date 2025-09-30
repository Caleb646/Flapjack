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
    // eDEVICE_ID_t deviceId    = device.deviceId;
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

        pOutBusInterface->read      = SPIRead_Blocking;
        pOutBusInterface->write     = SPIWrite_Blocking;
        pOutBusInterface->writeRead = SPIWriteRead_Blocking;
        return eSTATUS_SUCCESS;
    }

    if (BUS_ID_IS_UART (busId)) {

        UART_INIT (&status, device, bus);
        if (STATUS_FAIL (status)) {
            return eSTATUS_FAILURE;
        }

        pOutBusInterface->read      = UARTRead_Blocking;
        pOutBusInterface->write     = UARTWrite_Blocking;
        pOutBusInterface->writeRead = NULL;
        return eSTATUS_SUCCESS;
    }


    return status;
}