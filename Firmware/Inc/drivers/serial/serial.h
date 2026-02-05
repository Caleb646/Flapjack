#ifndef DRIVERS_SERIAL_H
#define DRIVERS_SERIAL_H

#include <stdint.h>

#include "common.h"

#include "drivers/serial/serial_defs.h"

FJ_DECLARE_SHARED (SerialPortVtable_t, e_UartVtbl);
FJ_DECLARE_SHARED (UartHwCfg_t, e_UartHwCfgs[]);
FJ_DECLARE_SHARED (UartDevice_t, e_UartDevices[]);
FJ_DECLARE_SHARED (uint8_t, e_nUartDevices);

FJ_DECLARE_SHARED (SerialPort_t*, e_pSerialPorts[eSERIAL_PORT_COUNT]);

eSTATUS_t Plat_Uart_Init (SerialPortCfg_t const* pCfg, SerialPort_t* pOutSerialPort);

UartDevice_t* Uart_GetByPortId (eSERIAL_PORT_ID_t portId);
UartHwCfg_t* Uart_GetHwCfgByPortId (eSERIAL_PORT_ID_t portId);

SerialPort_t* SerialPort_GetById (eSERIAL_PORT_ID_t portId);
SerialPort_t* SerialPort_Init (SerialPortCfg_t const* pCfg);
eSTATUS_t SerialPort_Write (SerialPort_t* pSerialPort, uint8_t const* pData, uint32_t size);
uint32_t SerialPort_Read (SerialPort_t* pSerialPort, uint8_t* pData, uint32_t size);
eSTATUS_t SerialPort_SetRxCallback (SerialPort_t* pPort, SerialRxCallback_t callback, void* pCallbackArg);

static inline void SerialPort_BeginCriticalSection (SerialPort_t* pSerialPort, SpinLock_t* pLock) {
    // TODO move to platform layer
    // TODO should check/save the state of the interrupt before enabling/disabling
    HAL_NVIC_DisableIRQ (pSerialPort->irqNum);
    if (pLock && pSerialPort->isShared) {
        SpinLock_Take (pLock);
    }
}

static inline void SerialPort_BeginCriticalSection_ISR (SerialPort_t* pSerialPort, SpinLock_t* pLock) {

    if (pLock && pSerialPort->isShared) {
        SpinLock_Take (pLock);
    }
}

static inline void SerialPort_EndCriticalSection (SerialPort_t* pSerialPort, SpinLock_t* pLock) {

    if (pLock && pSerialPort->isShared) {
        SpinLock_Release (pLock);
    }
    HAL_NVIC_EnableIRQ (pSerialPort->irqNum);
}

static inline void SerialPort_EndCriticalSection_ISR (SerialPort_t* pSerialPort, SpinLock_t* pLock) {

    if (pLock && pSerialPort->isShared) {
        SpinLock_Release (pLock);
    }
}

static inline bool SerialPort_TxHasDma (SerialPort_t* pSerialPort) {
    return (pSerialPort && pSerialPort->pTxDmaDev != NULL && RingBuffIsValid (&pSerialPort->txRingBuff));
}

static inline bool SerialPort_RxHasDma (SerialPort_t* pSerialPort) {
    return (pSerialPort && pSerialPort->pRxDmaDev != NULL && RingBuffIsValid (&pSerialPort->rxRingBuff));
}


#endif // DRIVERS_SERIAL_H