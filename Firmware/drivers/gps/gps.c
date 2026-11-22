
#include "core/core.h"

#include "target.h"

#include "drivers/gps/gpsdrv.h"
#include "drivers/gps/parser/minmea.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define GPS_SENTENCE_MAX_LENGTH MINMEA_MAX_SENTENCE_LENGTH
#define GPS_START_BYTE          '$'
#define GPS_END_BYTE            '\n' // last two bytes are \r\n

#define GPS_UBX_MAX_FRAME_SIZE 14U
#define GPS_UBX_LENGTH_IDX     4U
#define GPS_UBX_OVERHEAD       8U // sync, class, id, length, checksum

/* UartPort_Write busy-waits ~1 ms per byte at 9600, and this task outranks mag
 * and baro, so it sleeps between frames rather than sending them as one burst. */
#define GPS_CONFIG_GAP_MS   80U
#define GPS_CONFIG_RETRY_US 5000000U

typedef struct GpsUart_s {
    UartPort_t port;
} GpsUart_t;

/*
 * u-blox M8 (BN-880): 5 Hz, RMC and GGA only. Those two need ~750 of the 960
 * bytes/s 9600 baud carries, so the port stays at the receiver's default and a
 * command that fails leaves a working 1 Hz GPS rather than a baud mismatch.
 *
 * The receiver keeps this in RAM and loses it on a power cycle.
 */
static uint8_t const s_UbxConfig[][GPS_UBX_MAX_FRAME_SIZE] = {
    { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A }, // CFG-RATE 200 ms
    { 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x01, 0xFB, 0x10 }, // GGA on
    { 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x01, 0xFF, 0x18 }, // RMC on
    { 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11 }, // GLL off
    { 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13 }, // GSA off
    { 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15 }, // GSV off
    { 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19 }, // VTG off
};

/*
 * The ISR assembles into s_AssemblyBuffer, which only it touches, and publishes
 * each finished sentence into s_LatestSentence under a sequence lock - the same
 * arrangement as drivers/rx/crsf.c. The receiver sends sentences back to back,
 * so the next one starts landing while Gps_Task is still parsing this one.
 *
 * s_SentenceSeq is odd while the ISR is writing s_LatestSentence, and each
 * completed publish bumps it by two.
 *
 * +1 so the sentence can always be NUL-terminated for minmea, which takes a
 * C string and will otherwise read whatever a previous longer sentence left.
 */
FJ_DEFINE_SHARED (uint8_t, s_AssemblyBuffer[GPS_SENTENCE_MAX_LENGTH + 1U]) = { 0 };
FJ_DEFINE_SHARED (uint8_t, s_LatestSentence[GPS_SENTENCE_MAX_LENGTH + 1U]) = { 0 };
FJ_DEFINE_SHARED (uint32_t, s_SentenceIndex)                               = 0;
FJ_DEFINE_SHARED (uint32_t volatile, s_SentenceSeq)                        = 0;
FJ_DEFINE_SHARED (uint32_t, s_ConsumedSentenceSeq)                         = 0;

/* Writer and reader are an ISR and a task on the same core, so ordering only
 * has to survive the compiler: a signal fence, not a hardware barrier. */
#define GPS_FENCE() __atomic_signal_fence (__ATOMIC_SEQ_CST)

/* File scope, not on GpsUart_t, because UartRxCallback_t carries no ctx - the
 * RX handler can only reach state that lives beside the assembler it feeds. */
FJ_DEFINE_SHARED (STATIC DataReadySignal_t, s_Signal) = { 0 };

FJ_DEFINE_SHARED (STATIC UartPort_t*, s_pGpsPort) = NULL;
FJ_DEFINE_SHARED (STATIC bool, s_HasSentConfig)   = false;
FJ_DEFINE_SHARED (STATIC uint32_t, s_usLastConfig) = 0;

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
        s_AssemblyBuffer[s_SentenceIndex++] = byte;
        if (byte == GPS_END_BYTE) {
            s_AssemblyBuffer[s_SentenceIndex] = '\0';
            s_SentenceSeq++;
            GPS_FENCE ();
            memcpy (s_LatestSentence, s_AssemblyBuffer, s_SentenceIndex + 1U);
            GPS_FENCE ();
            s_SentenceSeq++;
            s_SentenceIndex = 0;
            /* A whole sentence is this receiver's "a sample landed" - signalling
             * per byte would wake the task ~80 times for one fix, and 79 of
             * those would find Gps_IsDataReady false. */
            if (s_Signal.Notify) {
                s_Signal.Notify (s_Signal.ctx);
            }
        }
    }
}

STATIC bool Gps_IsDataReady (void* ctx) {

    (void)ctx;
    return s_SentenceSeq != s_ConsumedSentenceSeq;
}

/*
 * Called when a sentence type this config turns off arrives, which means the
 * receiver is at its defaults. Waiting for that rather than sending at init
 * covers a receiver still booting and one that browned out in flight, and a
 * configured receiver or the SIL (RMC and GGA only) never reaches it.
 */
static void Gps_ConfigureReceiver (void) {

    uint32_t const usNow = GetMicroseconds ();
    if (s_HasSentConfig && (usNow - s_usLastConfig) < GPS_CONFIG_RETRY_US) {
        return;
    }
    s_HasSentConfig = true;
    s_usLastConfig  = usNow;

    LOG_INFO ("GPS receiver at defaults; sending 5 Hz RMC/GGA config");
    for (uint32_t i = 0; i < (sizeof (s_UbxConfig) / sizeof (s_UbxConfig[0])); ++i) {
        uint8_t const* pFrame = s_UbxConfig[i];
        (void)UartPort_Write (s_pGpsPort, pFrame, pFrame[GPS_UBX_LENGTH_IDX] + GPS_UBX_OVERHEAD);
        Delay (GPS_CONFIG_GAP_MS);
    }
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

    char sentence[GPS_SENTENCE_MAX_LENGTH + 1U];
    uint32_t seq;
    do {
        seq = s_SentenceSeq;
        if (seq == s_ConsumedSentenceSeq) {
            return eSTATUS_FAILURE;
        }
        GPS_FENCE ();
        memcpy (sentence, s_LatestSentence, sizeof (sentence));
        GPS_FENCE ();
    } while (((seq & 1U) != 0U) || (seq != s_SentenceSeq));
    s_ConsumedSentenceSeq = seq;

    enum minmea_sentence_id sentenceId = minmea_sentence_id (sentence, false);
    if (sentenceId == MINMEA_SENTENCE_GLL || sentenceId == MINMEA_SENTENCE_GSA ||
        sentenceId == MINMEA_SENTENCE_GSV || sentenceId == MINMEA_SENTENCE_VTG) {
        Gps_ConfigureReceiver ();
    }

    switch (sentenceId) {
    case MINMEA_SENTENCE_RMC: {
        struct minmea_sentence_rmc rmc;
        if (!minmea_parse_rmc (&rmc, sentence)) {
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
        /* The receiver sends invalid fields empty, and course is commonly invalid
         * at a standstill. minmea reads empty as NaN, which would stick in nav's
         * horizontal filter until reboot, so these hold their last value. */
        float const speedKnots = minmea_tofloat (&rmc.speed);
        float const course     = minmea_tofloat (&rmc.course);
        if (!isnan (speedKnots)) {
            /* RMC speed is knots; GpsData_t is m/s. */
            pOutData->speed = speedKnots * 0.514444f;
        }
        if (!isnan (course)) {
            pOutData->course = course;
        }
        pOutData->usLastFix = GetMicroseconds ();
        if (pOutData->fixType == 0U) {
            pOutData->fixType = 2U;   // RMC alone cannot distinguish 2D from 3D
        }
        break;
    }
    case MINMEA_SENTENCE_GGA: {
        struct minmea_sentence_gga gga;
        if (!minmea_parse_gga (&gga, sentence)) {
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
        if (!minmea_parse_gsv (&frame, sentence)) {
            return eSTATUS_FAILURE;
        }
        return eSTATUS_UNSUPPORTED;
    }
    case MINMEA_SENTENCE_GST: {
        struct minmea_sentence_gst frame;
        if (!minmea_parse_gst (&frame, sentence)) {
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
    /* No memset of pOutDriver: cfg is the caller's input and lives in the same
     * struct, so clearing it here would erase what this function is reading. */
    pOutDriver->ctx = Allocate(sizeof(GpsUart_t));
    if (!pOutDriver->ctx) {
        return eSTATUS_FAILURE;
    }

    /* Before UartPort_Init, which enables the RX interrupt: assigning after it
     * races the first byte, and a sentence that completes in that window is
     * assembled but never signalled. */
    s_Signal = pOutDriver->cfg.signal;

    GpsUart_t* pGpsUart = (GpsUart_t*)pOutDriver->ctx;
    pGpsUart->port.cfg.id       = BRD_GET_UART_ID (GPS);
    pGpsUart->port.cfg.baudRate = BRD_GET_BAUD_RATE (GPS);
    /* The signal owns this: its Notify is what runs at this priority and what
     * has to stay at or below the kernel's syscall ceiling. 5 with no signal
     * only keeps the polling build where it already was. */
    pGpsUart->port.cfg.irqPriority = s_Signal.Notify ? s_Signal.irqPriority : 5;
    pGpsUart->port.cfg.rxCallback  = Gps_DataReceivedHandler;
    pGpsUart->port.cfg.rxFifo      = true;
    if (UartPort_Init (&pGpsUart->port) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize UART port for GPS");
        return eSTATUS_FAILURE;
    }
    s_pGpsPort = &pGpsUart->port;

    pOutDriver->Read        = Gps_Read;
    pOutDriver->IsDataReady = Gps_IsDataReady;
    return eSTATUS_SUCCESS;
#endif
}