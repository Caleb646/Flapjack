
#include "core/core.h"

#include "drivers/sensors/gps/gps.h"
#include "drivers/sensors/gps/parser/minmea.h"

#include <stdbool.h>
#include <stdint.h>

#define GPS_SENTENCE_MAX_LENGTH MINMEA_MAX_SENTENCE_LENGTH
#define GPS_START_BYTE          '$'
#define GPS_END_BYTE            '\n' // last two bytes are \r\n

// TODO: do static setup of uart for gps including rx callback
FJ_DEFINE_SHARED (Gps_t, g_Gps)                                       = { 0 };
FJ_DEFINE_SHARED (uint8_t, s_SentenceBuffer[GPS_SENTENCE_MAX_LENGTH]) = { 0 };
FJ_DEFINE_SHARED (uint32_t, s_SentenceIndex)                          = 0;
FJ_DEFINE_SHARED (bool, s_IsSentenceReady)                            = false;

void Gps_DataReceivedHandler (uint8_t const* pData, uint32_t size) {

    // The gStartByte should always be GPS_START_BYTE when we get here
    uint8_t byte = pData[0];
    if (s_SentenceIndex == 0U && byte != GPS_START_BYTE) {
        return;
    }
    if (s_SentenceIndex >= GPS_SENTENCE_MAX_LENGTH) {
        s_SentenceIndex = 0;
        return;
    }
    s_SentenceBuffer[s_SentenceIndex++] = byte;
    if (byte == GPS_END_BYTE) {
        s_SentenceIndex   = 0;
        s_IsSentenceReady = true;
    }
}

eSTATUS_t Gps_Init_ (Gps_t* pOutGps) {

    if (UartPort_Init (&pOutGps->uartPort) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize UART port for GPS");
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t Gps_Update_ (Gps_t* pGps, GpsData_t* pOutData) {

    if (!s_IsSentenceReady) {
        return eSTATUS_FAILURE;
    }
    s_IsSentenceReady                  = false;
    enum minmea_sentence_id sentenceId = minmea_sentence_id ((char const*)s_SentenceBuffer, false);
    switch (sentenceId) {
    case MINMEA_SENTENCE_RMC: {
        struct minmea_sentence_rmc rmc;
        if (minmea_parse_rmc (&rmc, (char const*)s_SentenceBuffer)) {

        } else {
            return eSTATUS_FAILURE;
        }
        break;
    }
    case MINMEA_SENTENCE_GGA: {
        struct minmea_sentence_gga gga;
        if (minmea_parse_gga (&gga, (char const*)s_SentenceBuffer)) {

        } else {
            return eSTATUS_FAILURE;
        }
        break;
    }
    case MINMEA_SENTENCE_GSV: {
        struct minmea_sentence_gsv frame;
        if (minmea_parse_gsv (&frame, (char const*)s_SentenceBuffer)) {
        } else {
            return eSTATUS_FAILURE;
        }
    } break;
    case MINMEA_SENTENCE_GST: {
        struct minmea_sentence_gst frame;
        if (minmea_parse_gst (&frame, (char const*)s_SentenceBuffer)) {
        } else {
            return eSTATUS_FAILURE;
        }
    } break;
    default: return eSTATUS_UNSUPPORTED;
    }
    return eSTATUS_SUCCESS;
}
