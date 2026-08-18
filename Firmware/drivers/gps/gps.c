
#include "core/core.h"

#include "target.h"

#include "drivers/gps/gpsdrv.h"
#include "drivers/gps/parser/minmea.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define GPS_SENTENCE_MAX_LENGTH MINMEA_MAX_SENTENCE_LENGTH
#define GPS_START_BYTE          '$'
#define GPS_END_BYTE            '\n' // last two bytes are \r\n

typedef struct GpsUart_s {
    UartPort_t port;
} GpsUart_t;

/* +1 so the sentence can always be NUL-terminated for minmea, which takes a
 * C string and will otherwise read whatever a previous longer sentence left. */
FJ_DEFINE_SHARED (uint8_t, s_SentenceBuffer[GPS_SENTENCE_MAX_LENGTH + 1U]) = { 0 };
FJ_DEFINE_SHARED (uint32_t, s_SentenceIndex)                              = 0;
FJ_DEFINE_SHARED (bool, s_IsSentenceReady)                                = false;

STATIC void Gps_DataReceivedHandler (uint8_t const* pData, uint32_t size) {

    /* The UART ISR currently hands over one byte at a time (uart.c), but the
     * signature promises a run of them - honour it, or a batching ISR would
     * silently drop all but the first byte. */
    for (uint32_t i = 0; i < size; ++i) {

        uint8_t byte = pData[i];

        // Resync: nothing before a '$' can be part of a sentence.
        if (s_SentenceIndex == 0U && byte != GPS_START_BYTE) {
            continue;
        }
        if (s_SentenceIndex >= GPS_SENTENCE_MAX_LENGTH) {
            s_SentenceIndex = 0;
            continue;
        }
        s_SentenceBuffer[s_SentenceIndex++] = byte;
        if (byte == GPS_END_BYTE) {
            s_SentenceBuffer[s_SentenceIndex] = '\0';
            s_SentenceIndex                   = 0;
            s_IsSentenceReady                 = true;
        }
    }
}

STATIC bool Gps_IsDataReady (void* ctx) {

    (void)ctx;
    return s_IsSentenceReady;
}

/*
 * DDMM.MMMM -> DD.DDDDDD in double.
 *
 * minmea_tocoord() does exactly this but returns float, which throws away the
 * precision GpsData_t and umsg_sensors_gps_t both keep. Same decomposition,
 * same NaN-on-unknown contract, done in double.
 */
static double Gps_ToCoord (struct minmea_float const* pField) {

    if (pField->scale == 0) {
        return (double)NAN;
    }
    if (pField->scale > (INT_LEAST32_MAX / 100) || pField->scale < (INT_LEAST32_MIN / 100)) {
        return (double)NAN;
    }
    int_least32_t degrees = pField->value / (pField->scale * 100);
    int_least32_t minutes = pField->value % (pField->scale * 100);
    return (double)degrees + (double)minutes / (60.0 * (double)pField->scale);
}

STATIC eSTATUS_t Gps_Read (void* ctx, bool forcePolling, GpsData_t* pOutData) {

    (void)ctx;
    (void)forcePolling;
    if (!pOutData) {
        return eSTATUS_NULL_ARG;
    }

    if (!s_IsSentenceReady) {
        return eSTATUS_FAILURE;
    }
    s_IsSentenceReady                  = false;
    enum minmea_sentence_id sentenceId = minmea_sentence_id ((char const*)s_SentenceBuffer, false);
    switch (sentenceId) {
    case MINMEA_SENTENCE_RMC: {
        struct minmea_sentence_rmc rmc;
        if (!minmea_parse_rmc (&rmc, (char const*)s_SentenceBuffer)) {
            return eSTATUS_FAILURE;
        }
        /* A void RMC still parses; it just carries no position. Reporting its
         * blank coordinates as a fix is how a receiver with no lock ends up
         * looking like one sitting at the equator. */
        if (!rmc.valid) {
            pOutData->fixType = 0U;
            return eSTATUS_UNSUPPORTED;
        }
        pOutData->latitude  = Gps_ToCoord (&rmc.latitude);
        pOutData->longitude = Gps_ToCoord (&rmc.longitude);
        /* RMC speed is knots; GpsData_t is m/s. */
        pOutData->speed     = minmea_tofloat (&rmc.speed) * 0.514444f;
        pOutData->course    = minmea_tofloat (&rmc.course);
        pOutData->usLastFix = GetMicroseconds ();
        if (pOutData->fixType == 0U) {
            pOutData->fixType = 2U;   // RMC alone cannot distinguish 2D from 3D
        }
        break;
    }
    case MINMEA_SENTENCE_GGA: {
        struct minmea_sentence_gga gga;
        if (!minmea_parse_gga (&gga, (char const*)s_SentenceBuffer)) {
            return eSTATUS_FAILURE;
        }
        if (gga.fix_quality <= 0) {
            pOutData->fixType         = 0U;
            pOutData->satellitesInUse = (uint8_t)gga.satellites_tracked;
            return eSTATUS_UNSUPPORTED;
        }
        pOutData->latitude        = Gps_ToCoord (&gga.latitude);
        pOutData->longitude       = Gps_ToCoord (&gga.longitude);
        pOutData->altitude        = minmea_tofloat (&gga.altitude);
        pOutData->satellitesInUse = (uint8_t)gga.satellites_tracked;
        pOutData->fixType         = 3U;   // GGA carries altitude, so treat as 3D
        pOutData->usLastFix       = GetMicroseconds ();
        break;
    }
    /*
     * Parsed for validation only - neither carries a position update, so they
     * must not return SUCCESS: devices/gps.c publishes on SUCCESS, and doing so
     * here would republish a stale fix and inflate the publish count.
     */
    case MINMEA_SENTENCE_GSV: {
        struct minmea_sentence_gsv frame;
        if (!minmea_parse_gsv (&frame, (char const*)s_SentenceBuffer)) {
            return eSTATUS_FAILURE;
        }
        return eSTATUS_UNSUPPORTED;
    }
    case MINMEA_SENTENCE_GST: {
        struct minmea_sentence_gst frame;
        if (!minmea_parse_gst (&frame, (char const*)s_SentenceBuffer)) {
            return eSTATUS_FAILURE;
        }
        return eSTATUS_UNSUPPORTED;
    }
    default: return eSTATUS_UNSUPPORTED;
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t GpsDrv_Init (GpsDriver_t* pOutDriver) {

    if (!pOutDriver) {
        return eSTATUS_NULL_ARG;
    }

    /*
     * Not every board has a receiver: nucleo_h747zi defines no GPS_UART, and an
     * unguarded BRD_GET_UART_ID(GPS) expands to the undeclared token GPS_UART_ID
     * and fails to compile there. Reporting UNSUPPORTED lets Gps_Task exit
     * cleanly, which is the truthful outcome for a board with no GPS fitted.
     */
#if !BRD_IS_ENABLED (GPS)
    return eSTATUS_UNSUPPORTED;
#else
    memset(pOutDriver, 0, sizeof(GpsDriver_t));
    pOutDriver->ctx = Allocate(sizeof(GpsUart_t));
    if (!pOutDriver->ctx) {
        return eSTATUS_FAILURE;
    }

    GpsUart_t* pGpsUart = (GpsUart_t*)pOutDriver->ctx;
    pGpsUart->port.cfg.id          = BRD_GET_UART_ID (GPS);
    pGpsUart->port.cfg.baudRate    = BRD_GET_BAUD_RATE (GPS);
    pGpsUart->port.cfg.irqPriority = 5;
    pGpsUart->port.cfg.rxCallback = Gps_DataReceivedHandler;
    if (UartPort_Init (&pGpsUart->port) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize UART port for GPS");
        return eSTATUS_FAILURE;
    }

    pOutDriver->Read        = Gps_Read;
    pOutDriver->IsDataReady = Gps_IsDataReady;
    return eSTATUS_SUCCESS;
#endif
}