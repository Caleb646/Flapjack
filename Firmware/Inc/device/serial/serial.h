#ifndef DEVICE_SERIAL_SERIAL_H
#define DEVICE_SERIAL_SERIAL_H

#include "conf/board.h"
#include "conf/conf.h"
#include "core/core.h"
#include "hal.h"
#include "peripheral/bus/bus.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>


typedef struct {
    DevDesc_t* pDevDesc;
} SerialDebugInitConf_t;

typedef struct {
    eDEVICE_ID_t deviceId;
    Bus_t bus;
    bool isInitialized;
} SerialDebug_t;

// typedef SerialDebug_t volatile vSerialDebug_t;
typedef SerialDebug_t vSerialDebug_t;

eSTATUS_t SerialDebugInit (SerialDebugInitConf_t conf, SerialDebug_t* pOutSerial);
eSTATUS_t SerialDebugStart (vSerialDebug_t* pSerial);
vSerialDebug_t const* SerialDebugGetActiveDevice (void);
vSerialDebug_t* SerialDebugGetMutableActiveDevice (void);
vSerialDebug_t SerialDebugCopyOfActiveDevice (void);

#define SERIAL_DEBUG_INIT(pSTATUS, pDEV_DESC) SerialDebugInit ({ .pDevDesc = (pDEV_DESC) }, NULL);

#endif // DEVICE_SERIAL_SERIAL_H