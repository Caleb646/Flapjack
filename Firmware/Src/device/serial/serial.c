#include "device/serial/serial.h"
#include "common.h"
#include "conf/conf.h"
#include "core/core.h"
#include "hal.h"
#include "peripheral/bus/bus.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>


#define SERIAL_VALID(pSERIAL) ((pSERIAL) != NULL && (pSERIAL)->isInitialized)

static SHARED_MEM_SECTION SerialDebug_t g_SerialDebug = { 0 };

static void SerialDebugSink (uint8_t const* pData, uint32_t len) {

    if (SERIAL_VALID (&g_SerialDebug)) {
        Bus_Write (&g_SerialDebug.bus, eBUS_OP_MODE_BLOCK, pData, len);
    }
}

eSTATUS_t SerialDebugInit (SerialDebugInitConf_t conf, SerialDebug_t* pOutSerial) {

    eSTATUS_t status    = eSTATUS_SUCCESS;
    DevDesc_t* pDevDesc = conf.pDevDesc;

    RETURN_IF_NULL (pDevDesc, eSTATUS_NULL_ARG, "Device description is NULL");
    RETURN_IF_NOT (DEV_DESC_HAS_BUS (pDevDesc), eSTATUS_INVALID_ARG, "Device description is wrong type");

    SerialDebug_t* pSerial = &g_SerialDebug;
    if (pOutSerial != NULL) {
        pSerial = pOutSerial;
    }

    if (pSerial->isInitialized) {
        return eSTATUS_ALREADY_INITED;
    }

    memset (pSerial, 0, sizeof (SerialDebug_t));
    pSerial->deviceId = DEV_DESC_GET_ID (pDevDesc);

    status = BUS_INIT (pDevDesc, &pSerial->bus);
    GOTO_IF (FJ_FAIL (status), error, "Failed to init bus for serial debug");

    if (!BUS_SUPPORTS_OP (&pSerial->bus, eBUS_OP_DIR_WRITE, eBUS_OP_MODE_BLOCK)) {
        goto error;
    }

    pSerial->isInitialized = true;
    LoggerAddSink (SerialDebugSink);
    return eSTATUS_SUCCESS;
error:
    memset (pSerial, 0, sizeof (SerialDebug_t));
    return eSTATUS_FAILURE;
}

eSTATUS_t SerialDebugStart (vSerialDebug_t* pSerial) {

    if (!SERIAL_VALID (pSerial)) {
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

vSerialDebug_t const* SerialDebugGetActiveDevice (void) {

    if (!SERIAL_VALID (&g_SerialDebug)) {
        return NULL;
    }
    return &g_SerialDebug;
}

vSerialDebug_t* SerialDebugGetMutableActiveDevice (void) {

    if (!SERIAL_VALID (&g_SerialDebug)) {
        return NULL;
    }
    return &g_SerialDebug;
}

vSerialDebug_t SerialDebugCopyOfActiveDevice (void) {
    return g_SerialDebug;
}
