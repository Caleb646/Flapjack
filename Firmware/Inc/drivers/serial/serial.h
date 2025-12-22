#ifndef DRIVERS_SERIAL_H
#define DRIVERS_SERIAL_H

#include <stdint.h>

#include "common.h"

#include "drivers/serial/serial_defs.h"

FJ_DECLARE_SHARED (SerialPortVtable_t, e_UartVtbl);
FJ_DECLARE_SHARED (UartHwCfg_t, e_UartHwCfgs[]);
FJ_DECLARE_SHARED (UartDevice_t, e_UartDevices[]);

FJ_DECLARE_SHARED (SerialPort_t*, e_pSerialPorts[eSERIAL_PORT_COUNT]);

eSTATUS_t Plat_Uart_Init (SerialPortCfg_t const* pCfg, SerialPort_t* pOutSerialPort);

SerialPort_t* SerialPort_Init (SerialPortCfg_t const* pCfg);


#endif // DRIVERS_SERIAL_H