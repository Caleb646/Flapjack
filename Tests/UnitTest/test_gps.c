/*
 * GPS receiver tests: the NMEA byte assembler and the minmea-backed decode in
 * drivers/gps/gps.c.
 *
 * The golden sentences below are produced by Scripts/sim/nmea.py - the same
 * encoder the SIL bridge puts on the GPS UART - so the host encoder and the
 * firmware parser cannot drift apart. Same arrangement as test_crsf.c.
 *
 * The regression this file mainly exists for: Gps_Update_ used to run the
 * minmea parse and then discard every field, leaving pOutData untouched while
 * still returning eSTATUS_SUCCESS. A fix looked exactly like no fix, forever,
 * and nothing downstream could tell. test_Gga_PopulatesOutData is the direct
 * guard on that, and test_VoidRmc_IsNotReportedAsAFix covers the inverse - a
 * receiver with no lock must not read as a fix at 0,0.
 *
 * Doubles are compared with fabs rather than TEST_ASSERT_DOUBLE_WITHIN because
 * Unity excludes double support by default (unity_internals.h).
 */

#include "drivers/gps/gpsdrv.h"

#include "unity/unity.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifndef UNIT_TEST
#error "UNIT_TEST should be defined in this file"
#endif

/* gps.c reaches the UART only through GpsDrv_Init. Nothing here drives a real
 * port - the assembler is fed directly, exactly as the RX ISR feeds it, and the
 * backend's Read/IsDataReady are called through the same signatures the vtable
 * binds - so this satisfies the linker instead of pulling in uart.c, which does
 * not build against stubs/hal_stub.c.
 *
 * Those backend functions are file-static in the firmware and reachable here
 * only because they are declared STATIC, which core_shared.h expands to nothing
 * under UNIT_TEST. Same idiom as drivers/mag/sim.c. */
eSTATUS_t UartPort_Init (UartPort_t* pOutPort) {
    (void)pOutPort;
    return eSTATUS_SUCCESS;
}

static uint8_t s_TxBytes[128];
static uint32_t s_TxLen = 0;

eSTATUS_t UartPort_Write (UartPort_t* pPort, uint8_t const* pData, uint32_t size) {
    (void)pPort;
    TEST_ASSERT_LESS_OR_EQUAL_UINT32 (sizeof (s_TxBytes), s_TxLen + size);
    memcpy (&s_TxBytes[s_TxLen], pData, size);
    s_TxLen += size;
    return eSTATUS_SUCCESS;
}

void Gps_DataReceivedHandler (uint8_t const* pData, uint32_t size);
eSTATUS_t Gps_Read (void* ctx, bool forcePolling, GpsData_t* pOutData);
bool Gps_IsDataReady (void* ctx);

/* The data-ready signal the RX handler raises. GpsDrv_Init copies cfg.signal
 * into it on the board; here it is written directly, for the same reason the
 * assembler is fed directly rather than through a UART. */
extern DataReadySignal_t s_Signal;
extern UartPort_t* s_pGpsPort;
extern bool s_HasSentConfig;

static UartPort_t s_TestPort;

/* --- golden sentences, from `python -m sim.nmea` --------------------------- */

/* 37.6189 N, 122.3750 W, 100.0 m, 9 satellites, quality 1. */
#define GOLDEN_GGA "$GPGGA,123519.00,3737.13400,N,12222.50000,W,1,09,0.9,100.0,M,0.0,M,,*40\r\n"
/* Same position, 5.0 m/s (9.719 kn) on a course of 42.50 deg, status A. */
#define GOLDEN_RMC "$GPRMC,123519.00,A,3737.13400,N,12222.50000,W,9.719,42.50,230625,,,A*46\r\n"

#define EXPECT_LAT  37.6189
#define EXPECT_LON  (-122.3750)
/* 5 decimal places on the minutes field resolves ~1.9 cm; assert well inside
 * that so a precision regression (e.g. narrowing back to float) fails here. */
#define COORD_EPS   1e-7

void setUp (void) {
    s_pGpsPort = &s_TestPort;
    /* Drain any partial sentence a previous test left in the assembler. */
    uint8_t const nl = '\n';
    Gps_DataReceivedHandler (&nl, 1U);
    GpsData_t scratch;
    (void)Gps_Read (NULL, false, &scratch);
    /* No signal by default: that is the polling configuration, which every
     * other test in this file exercises. */
    memset (&s_Signal, 0, sizeof (s_Signal));
    s_HasSentConfig = false;
    s_TxLen         = 0;
}

void tearDown (void) {
}

static void Feed (char const* pSentence) {
    Gps_DataReceivedHandler ((uint8_t const*)pSentence, (uint32_t)strlen (pSentence));
}

static void AssertClose (double expected, double actual, double eps, char const* pWhat) {
    TEST_ASSERT_TRUE_MESSAGE (fabs (expected - actual) < eps, pWhat);
}

/* --- the assembler --------------------------------------------------------- */

static void test_NoSentence_UpdateFails (void) {
    GpsData_t out = { 0 };
    TEST_ASSERT_NOT_EQUAL (eSTATUS_SUCCESS, Gps_Read (NULL, false, &out));
}

static void test_PartialSentence_IsNotReadyUntilNewline (void) {
    GpsData_t out = { 0 };
    Feed ("$GPGGA,123519.00,3737.13400,N,12222.50000,W,1,09,0.9,100.0,M,0.0,M,,*40");
    TEST_ASSERT_NOT_EQUAL_MESSAGE (eSTATUS_SUCCESS, Gps_Read (NULL, false, &out),
                                   "sentence must not be decodable before its newline");
    Feed ("\r\n");
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Gps_Read (NULL, false, &out));
}

static void test_LeadingGarbage_IsResynced (void) {
    GpsData_t out = { 0 };
    /* A receiver powering up mid-sentence, or line noise. Everything before the
     * '$' has to be dropped rather than prefixed onto the sentence. */
    Feed ("\xFF\x00qrs junk");
    Feed (GOLDEN_GGA);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Gps_Read (NULL, false, &out));
    AssertClose (EXPECT_LAT, out.latitude, COORD_EPS, "latitude after resync");
}

static void test_ByteAtATime_MatchesBulkDelivery (void) {
    /* uart.c hands over one byte per RXNE interrupt, but the callback signature
     * promises a run. Both paths must assemble the same sentence. */
    GpsData_t out = { 0 };
    char const* p = GOLDEN_GGA;
    for (uint32_t i = 0; i < strlen (GOLDEN_GGA); ++i) {
        uint8_t byte = (uint8_t)p[i];
        Gps_DataReceivedHandler (&byte, 1U);
    }
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Gps_Read (NULL, false, &out));
    AssertClose (EXPECT_LAT, out.latitude, COORD_EPS, "latitude, byte-at-a-time");
    AssertClose (EXPECT_LON, out.longitude, COORD_EPS, "longitude, byte-at-a-time");
}

/* --- decode ---------------------------------------------------------------- */

static void test_Gga_PopulatesOutData (void) {
    GpsData_t out = { 0 };
    Feed (GOLDEN_GGA);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Gps_Read (NULL, false, &out));

    /* The discarded-parse regression: these were all still 0 while the function
     * returned SUCCESS. */
    AssertClose (EXPECT_LAT, out.latitude, COORD_EPS, "GGA latitude");
    AssertClose (EXPECT_LON, out.longitude, COORD_EPS, "GGA longitude");
    TEST_ASSERT_FLOAT_WITHIN (0.05f, 100.0f, out.altitude);
    TEST_ASSERT_EQUAL_UINT8 (9U, out.satellitesInUse);
    TEST_ASSERT_EQUAL_UINT8 (3U, out.fixType);
}

static void test_Rmc_PopulatesSpeedAndCourse (void) {
    GpsData_t out = { 0 };
    Feed (GOLDEN_RMC);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Gps_Read (NULL, false, &out));

    AssertClose (EXPECT_LAT, out.latitude, COORD_EPS, "RMC latitude");
    AssertClose (EXPECT_LON, out.longitude, COORD_EPS, "RMC longitude");
    /* On the wire in knots; GpsData_t is m/s. Getting this wrong is a silent
     * 1.94x error in every speed the estimator would ever see. */
    TEST_ASSERT_FLOAT_WITHIN (0.01f, 5.0f, out.speed);
    TEST_ASSERT_FLOAT_WITHIN (0.01f, 42.5f, out.course);
}

static void test_Rmc_EmptyCourse_KeepsLastCourse (void) {
    GpsData_t out = { 0 };
    Feed (GOLDEN_RMC);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Gps_Read (NULL, false, &out));

    /* A stationary u-blox M8 sends an empty course. Passed on as NaN it reaches
     * nav's horizontal filter, whose states then stay NaN until reboot. */
    Feed ("$GNRMC,123519.00,A,3737.13400,N,12222.50000,W,0.012,,230625,,,A*70\r\n");
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Gps_Read (NULL, false, &out));
    TEST_ASSERT_FLOAT_WITHIN (0.01f, 42.5f, out.course);
    TEST_ASSERT_FLOAT_WITHIN (0.001f, 0.012f * 0.514444f, out.speed);
}

static void test_Rmc_EmptySpeed_KeepsLastSpeed (void) {
    GpsData_t out = { 0 };
    Feed (GOLDEN_RMC);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Gps_Read (NULL, false, &out));

    Feed ("$GNRMC,123519.00,A,3737.13400,N,12222.50000,W,,,230625,,,A*5D\r\n");
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Gps_Read (NULL, false, &out));
    TEST_ASSERT_FLOAT_WITHIN (0.01f, 5.0f, out.speed);
    TEST_ASSERT_FLOAT_WITHIN (0.01f, 42.5f, out.course);
}

static void test_NextSentenceArriving_DoesNotCorruptLatest (void) {
    GpsData_t out = { 0 };
    /* The receiver sends sentences back to back, so the head of the next one
     * can land before the task has read this one. */
    Feed (GOLDEN_GGA);
    Feed ("$GPRMC,999999.00,A,0000");
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Gps_Read (NULL, false, &out));
    AssertClose (EXPECT_LAT, out.latitude, COORD_EPS, "latitude with next sentence in flight");
    TEST_ASSERT_EQUAL_UINT8 (9U, out.satellitesInUse);
}

static void test_VoidRmc_IsNotReportedAsAFix (void) {
    GpsData_t out = { 0 };
    /* Status 'V' - receiver has no lock. Its coordinate fields are meaningless,
     * so publishing them would put the vehicle at a plausible-looking fix. */
    Feed ("$GPRMC,123519.00,V,0000.00000,N,00000.00000,E,0.000,0.00,230625,,,N*7B\r\n");
    TEST_ASSERT_NOT_EQUAL_MESSAGE (eSTATUS_SUCCESS, Gps_Read (NULL, false, &out),
                                   "a void RMC must not publish as a fix");
    TEST_ASSERT_EQUAL_UINT8 (0U, out.fixType);
}

static void test_GgaNoFix_IsNotReportedAsAFix (void) {
    GpsData_t out = { 0 };
    /* fix_quality 0 = no position. Satellite count is still meaningful. */
    Feed ("$GPGGA,123519.00,0000.00000,N,00000.00000,E,0,03,99.9,0.0,M,0.0,M,,*6A\r\n");
    TEST_ASSERT_NOT_EQUAL_MESSAGE (eSTATUS_SUCCESS, Gps_Read (NULL, false, &out),
                                   "GGA with fix_quality 0 must not publish as a fix");
    TEST_ASSERT_EQUAL_UINT8 (0U, out.fixType);
    TEST_ASSERT_EQUAL_UINT8 (3U, out.satellitesInUse);
}

static void test_NonPositionalSentence_DoesNotRepublish (void) {
    GpsData_t out = { 0 };
    /* GSV carries satellites in view and no position. Returning SUCCESS here
     * would make devices/gps.c republish a stale fix and inflate gps_count,
     * which is the metric the SIL's loopback check reads. */
    Feed ("$GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74\r\n");
    TEST_ASSERT_NOT_EQUAL (eSTATUS_SUCCESS, Gps_Read (NULL, false, &out));
}

static void test_BadChecksum_IsRejected (void) {
    GpsData_t out = { 0 };
    Feed ("$GPGGA,123519.00,3737.13400,N,12222.50000,W,1,09,0.9,100.0,M,0.0,M,,*00\r\n");
    TEST_ASSERT_NOT_EQUAL (eSTATUS_SUCCESS, Gps_Read (NULL, false, &out));
}

/* --- the data-ready signal -------------------------------------------------- */

static uint32_t s_NotifyCount = 0;
static void* s_NotifyCtx      = NULL;

static void CountingNotify (void* ctx) {
    ++s_NotifyCount;
    s_NotifyCtx = ctx;
}

static void InstallSignal (void) {
    s_NotifyCount = 0;
    s_NotifyCtx   = NULL;
    s_Signal.Notify = CountingNotify;
    s_Signal.ctx    = &s_NotifyCount;
}

static void test_CompleteSentence_RaisesSignalOnce (void) {
    InstallSignal ();
    Feed (GOLDEN_GGA);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE (1U, s_NotifyCount, "one sentence, one notification");
    TEST_ASSERT_EQUAL_PTR_MESSAGE (&s_NotifyCount, s_NotifyCtx, "ctx must reach Notify unchanged");
}

static void test_PartialSentence_RaisesNoSignal (void) {
    /* The task must not be woken for a sentence Gps_Read would reject: it is the
     * terminator that makes one readable, not the arrival of bytes. */
    InstallSignal ();
    Feed ("$GPGGA,123519.00,3737.13400,N,12222.50000,W,1,09,0.9,100.0,M,0.0,M,,*40");
    TEST_ASSERT_EQUAL_UINT32 (0U, s_NotifyCount);
    Feed ("\r\n");
    TEST_ASSERT_EQUAL_UINT32 (1U, s_NotifyCount);
}

static void test_ByteAtATime_RaisesOneSignalPerSentence (void) {
    /* uart.c delivers one byte per RXNE interrupt, so this is the real path - a
     * notification per byte would wake the task ~80 times for one sentence. */
    InstallSignal ();
    for (uint32_t i = 0; i < strlen (GOLDEN_GGA); ++i) {
        uint8_t byte = (uint8_t)GOLDEN_GGA[i];
        Gps_DataReceivedHandler (&byte, 1U);
    }
    TEST_ASSERT_EQUAL_UINT32 (1U, s_NotifyCount);
}

static void test_NoSignal_StillAssembles (void) {
    /* A NULL Notify means "the caller polls" and must not change the assembler. */
    GpsData_t out = { 0 };
    Feed (GOLDEN_GGA);
    TEST_ASSERT_TRUE (Gps_IsDataReady (NULL));
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Gps_Read (NULL, false, &out));
}

/* --- receiver configuration ------------------------------------------------- */

/* Walks s_TxBytes as UBX frames, checking sync and checksum, and returns the
 * rate each NMEA sentence was set to (-1 if no frame mentioned it). */
static void AssertConfigFrames (int outRates[6], uint32_t* pOutMeasRateMs) {
    for (uint32_t i = 0; i < 6U; ++i) {
        outRates[i] = -1;
    }
    *pOutMeasRateMs = 0;

    uint32_t offset = 0;
    while (offset < s_TxLen) {
        uint8_t const* pFrame = &s_TxBytes[offset];
        TEST_ASSERT_LESS_OR_EQUAL_UINT32 (s_TxLen, offset + 8U);
        TEST_ASSERT_EQUAL_HEX8 (0xB5, pFrame[0]);
        TEST_ASSERT_EQUAL_HEX8 (0x62, pFrame[1]);
        uint32_t const payloadLen = (uint32_t)pFrame[4] | ((uint32_t)pFrame[5] << 8);
        uint32_t const frameLen   = payloadLen + 8U;
        TEST_ASSERT_LESS_OR_EQUAL_UINT32 (s_TxLen, offset + frameLen);

        uint8_t ckA = 0;
        uint8_t ckB = 0;
        for (uint32_t j = 2; j < frameLen - 2U; ++j) {
            ckA = (uint8_t)(ckA + pFrame[j]);
            ckB = (uint8_t)(ckB + ckA);
        }
        TEST_ASSERT_EQUAL_HEX8 (ckA, pFrame[frameLen - 2U]);
        TEST_ASSERT_EQUAL_HEX8 (ckB, pFrame[frameLen - 1U]);

        TEST_ASSERT_EQUAL_HEX8 (0x06, pFrame[2]);
        if (pFrame[3] == 0x08) {
            TEST_ASSERT_EQUAL_UINT32 (6U, payloadLen);
            *pOutMeasRateMs = (uint32_t)pFrame[6] | ((uint32_t)pFrame[7] << 8);
        } else {
            TEST_ASSERT_EQUAL_HEX8 (0x01, pFrame[3]);
            TEST_ASSERT_EQUAL_UINT32 (3U, payloadLen);
            TEST_ASSERT_EQUAL_HEX8 (0xF0, pFrame[6]);
            TEST_ASSERT_LESS_THAN_UINT8 (6U, pFrame[7]);
            outRates[pFrame[7]] = pFrame[8];
        }
        offset += frameLen;
    }
}

static void test_DefaultSentence_SendsConfig (void) {
    static char const* const defaults[] = {
        "$GNGLL,3737.13400,N,12222.50000,W,123519.00,A,A*6A\r\n",
        "$GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*2E\r\n",
        "$GPGSV,1,1,01,03,,,07*7C\r\n",
        "$GNVTG,,T,,M,0.012,N,0.022,K,A*3E\r\n",
    };

    for (uint32_t i = 0; i < (sizeof (defaults) / sizeof (defaults[0])); ++i) {
        GpsData_t out = { 0 };
        s_HasSentConfig = false;
        s_TxLen         = 0;
        Feed (defaults[i]);
        (void)Gps_Read (NULL, false, &out);

        int rates[6];
        uint32_t measRateMs;
        AssertConfigFrames (rates, &measRateMs);
        TEST_ASSERT_EQUAL_UINT32 (200U, measRateMs);
        TEST_ASSERT_EQUAL_INT (1, rates[0]); // GGA
        TEST_ASSERT_EQUAL_INT (0, rates[1]); // GLL
        TEST_ASSERT_EQUAL_INT (0, rates[2]); // GSA
        TEST_ASSERT_EQUAL_INT (0, rates[3]); // GSV
        TEST_ASSERT_EQUAL_INT (1, rates[4]); // RMC
        TEST_ASSERT_EQUAL_INT (0, rates[5]); // VTG
    }
}

static void test_ConfigIsNotRepeatedWithinRetryPeriod (void) {
    GpsData_t out = { 0 };
    Feed ("$GPGSV,1,1,01,03,,,07*7C\r\n");
    (void)Gps_Read (NULL, false, &out);
    uint32_t const sent = s_TxLen;
    TEST_ASSERT_GREATER_THAN_UINT32 (0U, sent);

    Feed ("$GNVTG,,T,,M,0.012,N,0.022,K,A*3E\r\n");
    (void)Gps_Read (NULL, false, &out);
    TEST_ASSERT_EQUAL_UINT32 (sent, s_TxLen);
}

static void test_ConfiguredOutput_SendsNothing (void) {
    /* RMC and GGA are all a configured receiver and the SIL ever send. */
    GpsData_t out = { 0 };
    Feed (GOLDEN_RMC);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Gps_Read (NULL, false, &out));
    Feed (GOLDEN_GGA);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Gps_Read (NULL, false, &out));
    TEST_ASSERT_EQUAL_UINT32 (0U, s_TxLen);
}

static void test_NullArgs_AreRejected (void) {
    GpsData_t out = { 0 };
    TEST_ASSERT_EQUAL (eSTATUS_NULL_ARG, Gps_Read (NULL, false, NULL));
    TEST_ASSERT_EQUAL (eSTATUS_NULL_ARG, GpsDrv_Init (NULL));
}

int main (void) {
    UNITY_BEGIN ();
    RUN_TEST (test_NoSentence_UpdateFails);
    RUN_TEST (test_PartialSentence_IsNotReadyUntilNewline);
    RUN_TEST (test_LeadingGarbage_IsResynced);
    RUN_TEST (test_ByteAtATime_MatchesBulkDelivery);
    RUN_TEST (test_Gga_PopulatesOutData);
    RUN_TEST (test_Rmc_PopulatesSpeedAndCourse);
    RUN_TEST (test_Rmc_EmptyCourse_KeepsLastCourse);
    RUN_TEST (test_Rmc_EmptySpeed_KeepsLastSpeed);
    RUN_TEST (test_NextSentenceArriving_DoesNotCorruptLatest);
    RUN_TEST (test_VoidRmc_IsNotReportedAsAFix);
    RUN_TEST (test_GgaNoFix_IsNotReportedAsAFix);
    RUN_TEST (test_NonPositionalSentence_DoesNotRepublish);
    RUN_TEST (test_BadChecksum_IsRejected);
    RUN_TEST (test_CompleteSentence_RaisesSignalOnce);
    RUN_TEST (test_PartialSentence_RaisesNoSignal);
    RUN_TEST (test_ByteAtATime_RaisesOneSignalPerSentence);
    RUN_TEST (test_NoSignal_StillAssembles);
    RUN_TEST (test_DefaultSentence_SendsConfig);
    RUN_TEST (test_ConfigIsNotRepeatedWithinRetryPeriod);
    RUN_TEST (test_ConfiguredOutput_SendsNothing);
    RUN_TEST (test_NullArgs_AreRejected);
    return UNITY_END ();
}
