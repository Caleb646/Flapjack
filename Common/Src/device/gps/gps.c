#include "device/gps/gps.h"
#include "common.h"
#include "device/gps/parser/minmea.h"
#include "log/logger.h"
#include "mem/mem.h"
#include "peripheral/bus/bus.h"
#include <stdbool.h>
#include <stdint.h>

#define GPS_SENTENCE_MAX_LENGTH MINMEA_MAX_SENTENCE_LENGTH
#define GPS_START_BYTE          '$'
#define GPS_END_BYTE            '\n' // last two bytes are \r\n
#define GPS_VALID(pGPS)         ((pGPS) != NULL && (pGPS)->isInitialized == true)

static SHARED_MEM_SECTION GPS_t gGPS                       = { 0 };
static uint8_t volatile gSentence[GPS_SENTENCE_MAX_LENGTH] = { 0 };
static uint8_t gSentenceCopy[GPS_SENTENCE_MAX_LENGTH]      = { 0 };
static uint32_t volatile gSentenceIndex                    = 0;
static bool volatile gSentenceReady                        = false;
static uint8_t volatile gStartByte                         = 0;

void GPSCallback (BusCallbackData_t data) {

    if (GPS_VALID (&gGPS) == false) {
        return;
    }

    if (data.cbId == eBUS_CALLBACK_ID_RX) {

        // The gStartByte should always be GPS_START_BYTE when we get here
        if (gStartByte != GPS_START_BYTE) {
            return;
        }
        gSentenceIndex              = 0;
        gSentenceReady              = false;
        gSentence[gSentenceIndex++] = gStartByte;

        // eSTATUS_t status = eSTATUS_SUCCESS;
        uint32_t timeout = 1000U;
        uint8_t byte     = 0;
        while (timeout-- > 0) {

            if (gSentenceIndex >= GPS_SENTENCE_MAX_LENGTH) {
                goto error;
            }

            if (BUS_READ_BLOCK (gGPS.bus, &byte, 1) != eSTATUS_SUCCESS) {
                goto error;
            }

            gSentence[gSentenceIndex++] = byte;
            if (byte == GPS_END_BYTE) {
                gSentenceReady = true;
                goto done;
            }
        }
    }

done:
error:
    gStartByte = 0;
    BUS_READ_IT (gGPS.bus, (uint8_t*)&gStartByte, 1U);
}

eSTATUS_t GPSInit (GPSInitConf_t conf, GPS_t* pOutGPS) {

    // bool success     = true;
    eSTATUS_t status = eSTATUS_SUCCESS;

    DeviceBoardConf_t deviceConf = conf.boardConf;
    eDEVICE_ID_t deviceId        = deviceConf.deviceId;
    BusBoardConf_t* pBusConf     = deviceConf.generic.pBusBoardConf;
    if (pBusConf == NULL) {
        return eSTATUS_NULL_ARG;
    }

    eBUS_ID_t busId = pBusConf->busId;

    vGPS_t* pGPS = &gGPS;
    if (pOutGPS != NULL) {
        pGPS = pOutGPS;
    }

    if (pGPS->isInitialized == true) {
        return eSTATUS_FAILURE;
    }

    memset (pGPS, 0, sizeof (GPS_t));
    pGPS->busId    = busId;
    pGPS->deviceId = deviceId;

    BUS_INIT (&status, deviceConf, *pBusConf, &pGPS->bus);

    if (pGPS->bus.ReadIT == NULL || pGPS->bus.ReadBlocking == NULL || pGPS->bus.RegisterCallback == NULL) {
        goto error;
    }

    BUS_REG_CALLBACK (&status, pGPS->bus, NULL, GPSCallback, eBUS_CALLBACK_ID_RX, 0);
    GOTO_IF (STATUS_FAIL (status), error, "Failed to register GPS RX callback");

    BUS_REG_CALLBACK (&status, pGPS->bus, NULL, GPSCallback, eBUS_CALLBACK_ID_ERROR, 0);
    GOTO_IF (STATUS_FAIL (status), error, "Failed to register GPS ERROR callback");

    pGPS->isInitialized = true;
    return status;

error:
    memset (pGPS, 0, sizeof (GPS_t));
    return status;
}

eSTATUS_t GPSStart (vGPS_t* pGPS) {

    if (GPS_VALID (pGPS) == false) {
        return eSTATUS_INVALID_ARG;
    }
    return BUS_READ_IT (pGPS->bus, (uint8_t*)&gStartByte, 1U);
}

eSTATUS_t GPSUpdate (vGPS_t* pGPS, GPSData_t* pOutData) {

    if (gSentenceReady == false) {
        return eSTATUS_BUSY;
    }

    ATOMIC_BLOCK_LOCAL (eNVIC_PRIO_LVL_MAX) {
        memcpy (gSentenceCopy, (void*)gSentence, GPS_SENTENCE_MAX_LENGTH);
    }

    enum minmea_sentence_id sentenceId = minmea_sentence_id ((char const*)gSentenceCopy, false);
    switch (sentenceId) {
    case MINMEA_SENTENCE_RMC: {
        struct minmea_sentence_rmc rmc;
        if (minmea_parse_rmc (&rmc, (char const*)gSentenceCopy)) {

        } else {
            return eSTATUS_FAILURE;
        }
        break;
    }
    case MINMEA_SENTENCE_GGA: {
        struct minmea_sentence_gga gga;
        if (minmea_parse_gga (&gga, (char const*)gSentenceCopy)) {

        } else {
            return eSTATUS_FAILURE;
        }
        break;
    }
    case MINMEA_SENTENCE_GSV: {
        struct minmea_sentence_gsv frame;
        if (minmea_parse_gsv (&frame, (char const*)gSentenceCopy)) {
        } else {
            return eSTATUS_FAILURE;
        }
    } break;
    case MINMEA_SENTENCE_GST: {
        struct minmea_sentence_gst frame;
        if (minmea_parse_gst (&frame, (char const*)gSentenceCopy)) {
        } else {
            return eSTATUS_FAILURE;
        }
    } break;
    default: return eSTATUS_UNSUPPORTED;
    }
    return eSTATUS_SUCCESS;
}
