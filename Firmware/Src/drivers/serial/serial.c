#include <stdint.h>

#include "common.h"

#include "drivers/serial/serial.h"

FJ_DEFINE_SHARED (SerialPort_t*, e_pSerialPorts[eSERIAL_PORT_COUNT]);

UartDevice_t* Uart_GetByPortId (eSERIAL_PORT_ID_t portId) {

    for (uint32_t i = 0; i < e_nUartDevices; ++i) {
        if (e_UartDevices[i].portId == portId) {
            return &e_UartDevices[i];
        }
    }
    return NULL;
}

UartHwCfg_t* Uart_GetHwCfgByPortId (eSERIAL_PORT_ID_t portId) {

    for (uint32_t i = 0; i < e_nUartDevices; ++i) {
        if (e_UartHwCfgs[i].portId == portId) {
            return &e_UartHwCfgs[i];
        }
    }
    return NULL;
}

FJ_TESTABLE SerialPort_t* SerialPort_Alloc (eSERIAL_PORT_ID_t portId) {

    if (!SERIAL_PORT_ID_VALID (portId)) {
        return NULL;
    }

    SerialPort_t* pSerialPort = (SerialPort_t*)Alloc_SharedMem (sizeof (SerialPort_t));
    if (!pSerialPort) {
        return NULL;
    }

    pSerialPort->portId = portId;

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

    if (pCfg->isShared) {
        pSerialPort->isShared = true;
        SpinLock_Init (&pSerialPort->rxLock);
        SpinLock_Init (&pSerialPort->txLock);
    }

    return pSerialPort;
}

eSTATUS_t SerialPort_Write (SerialPort_t* pSerialPort, uint8_t const* pData, uint32_t size) {

    if (!pSerialPort || !pData || !size) {
        return eSTATUS_FAIL;
    }

    if (pSerialPort->pVtbl && pSerialPort->pVtbl->fnWrite) {
        return pSerialPort->pVtbl->fnWrite (pSerialPort, pData, size);
    }
    return eSTATUS_FAIL;
}

uint32_t SerialPort_Read (SerialPort_t* pSerialPort, uint8_t* pData, uint32_t size) {

    if (!pSerialPort || !pData || !size) {
        return 0;
    }

    if (pSerialPort->pVtbl && pSerialPort->pVtbl->fnRead) {
        return pSerialPort->pVtbl->fnRead (pSerialPort, pData, size);
    }
    return 0;
}

eSTATUS_t SerialPort_SetRxCallback (SerialPort_t* pPort, SerialRxCallback_t callback, void* pCallbackArg) {

    if (!pPort) {
        return eSTATUS_FAIL;
    }

    pPort->fnRxCallback   = callback;
    pPort->pRxCallbackArg = pCallbackArg;
    return eSTATUS_OK;
}