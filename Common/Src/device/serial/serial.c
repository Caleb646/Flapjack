#include "device/serial/serial.h"
#include "common.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "hal.h"
#include "log/format.h"
#include "log/logger.h"
#include "mem/mem.h"
#include "peripheral/bus/bus.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SERIAL_VALID(pSERIAL) \
    ((pSERIAL) != NULL && (pSERIAL)->isInitialized == true && (pSERIAL)->bus.WriteBlocking != NULL)
static SHARED_MEM_SECTION SerialDebug_t g_SerialDebug = { 0 };

static void SerialDebugSink (uint8_t const* pData, uint32_t len) {

    if (SERIAL_VALID (&g_SerialDebug)) {
        BUS_WRITE_BLOCK (g_SerialDebug.bus, pData, len);
    }
}

eSTATUS_t SerialDebugInit (SerialDebugInitConf_t conf, SerialDebug_t* pOutSerial) {

    eSTATUS_t status             = eSTATUS_SUCCESS;
    DeviceBoardConf_t deviceConf = conf.boardConf;
    eDEVICE_ID_t deviceId        = deviceConf.deviceId;
    BusBoardConf_t* pBusConf     = deviceConf.generic.pBusBoardConf;
    if (pBusConf == NULL) {
        return eSTATUS_FAILURE;
    }

    eBUS_ID_t busId = pBusConf->busId;

    SerialDebug_t* pSerial = &g_SerialDebug;
    if (pOutSerial != NULL) {
        pSerial = pOutSerial;
    }

    if (pSerial->isInitialized == true) {
        return eSTATUS_FAILURE;
    }

    memset (pSerial, 0, sizeof (SerialDebug_t));
    pSerial->busId    = busId;
    pSerial->deviceId = deviceId;

    BUS_INIT (&status, deviceConf, *pBusConf, &pSerial->bus);

    if (pSerial->bus.WriteBlocking == NULL) {
        goto error;
    }

    pSerial->isInitialized = true;
    LoggerAddSink (SerialDebugSink);
    return eSTATUS_SUCCESS;
error:
    memset (pSerial, 0, sizeof (SerialDebug_t));
    return eSTATUS_FAILURE;
}
