#include <stdint.h>

#include "common.h"

#include "drivers/serial/serial.h"

FJ_DEFINE_SHARED (SerialPort_t*, e_pSerialPorts[eSERIAL_PORT_COUNT]);

SerialPort_t* SerialPort_Alloc (eSERIAL_PORT_ID_t portId) {

    if (portId == eSERIAL_PORT_ID_NULL || portId >= eSERIAL_PORT_COUNT) {
        return NULL;
    }

    SerialPort_t* pSerialPort = (SerialPort_t*)Alloc_SharedMem (sizeof (SerialPort_t));
    if (!pSerialPort) {
        return NULL;
    }

    pSerialPort->portId         = portId;
    pSerialPort->mode           = eSERIAL_PORT_MODE_TX_RX;
    pSerialPort->baudrate       = eSERIAL_PORT_BAUD_115200;
    pSerialPort->functions      = eSERIAL_PORT_FUNCTION_RX_SERIAL | eSERIAL_PORT_FUNCTION_TX_SERIAL;
    pSerialPort->fnRxCallback   = NULL;
    pSerialPort->pRxCallbackArg = NULL;

    e_pSerialPorts[SERIAL_PORT_ID_TO_INDEX (portId)] = pSerialPort;
    return pSerialPort;
}

SerialPort_t* SerialPort_GetById (eSERIAL_PORT_ID_t portId) {

    if (!SERIAL_PORT_ID_VALID (portId)) {
        return NULL;
    }
    return e_pSerialPorts[SERIAL_PORT_ID_TO_INDEX (portId)];
}

SerialPort_t* SerialPort_Init (SerialPortCfg_t const* pCfg) {

    if (!pCfg) {
        return NULL;
    }

    SerialPort_t* pSerialPort = SerialPort_GetById (pCfg->portId);
    if (pSerialPort) {
        return pSerialPort;
    }

    pSerialPort = SerialPort_Alloc (pCfg->portId);
    if (!pSerialPort) {
        return NULL;
    }

    eSTATUS_t status = eSTATUS_OK;
    switch (pCfg->portType) {
    case eSERIAL_PORT_TYPE_UART: status = Plat_Uart_Init (pCfg, pSerialPort); break;
    case eSERIAL_PORT_TYPE_USB_VCP:
    default: return NULL;
    }

    if (FJ_FAIL (status)) {
        return NULL;
    }

    return pSerialPort;
}

eSTATUS_t SerialPort_SetRxCallback (SerialPort_t* pPort, SerialRxCallback_t callback, void* pCallbackArg) {

    if (!pPort) {
        return eSTATUS_FAIL;
    }

    pPort->fnRxCallback   = callback;
    pPort->pRxCallbackArg = pCallbackArg;
    return eSTATUS_OK;
}