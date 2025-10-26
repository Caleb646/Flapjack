#ifndef DEVICE_SERIAL_SERIAL_H
#define DEVICE_SERIAL_SERIAL_H

#include "common.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "hal.h"
#include "log/format.h"
#include "peripheral/bus/bus.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>


typedef struct {
    DeviceBoardConf_t boardConf;
} SerialDebugInitConf_t;

typedef struct {
    eBUS_ID_t busId;
    eDEVICE_ID_t deviceId;
    BusVTable_t bus;
    bool isInitialized;
} SerialDebug_t;

// typedef SerialDebug_t volatile vSerialDebug_t;
typedef SerialDebug_t vSerialDebug_t;

eSTATUS_t SerialDebugInit (SerialDebugInitConf_t conf, SerialDebug_t* pOutSerial);
eSTATUS_t SerialDebugStart (vSerialDebug_t* pSerial);
vSerialDebug_t const* SerialDebugGetActiveDevice (void);
vSerialDebug_t* SerialDebugGetMutableActiveDevice (void);
vSerialDebug_t SerialDebugCopyOfActiveDevice (void);

#define SERIAL_DEBUG_INIT(pSTATUS, DEVICE_BOARD_CONF)              \
    do {                                                           \
        SerialDebugInitConf_t conf = { 0 };                        \
        conf.boardConf             = (DEVICE_BOARD_CONF);          \
        *(pSTATUS)                 = SerialDebugInit (conf, NULL); \
    } while (0)

#endif // DEVICE_SERIAL_SERIAL_H