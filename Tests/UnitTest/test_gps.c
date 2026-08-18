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

eSTATUS_t UartPort_Write (UartPort_t* pPort, uint8_t const* pData, uint32_t size) {
    (void)pPort;
    (void)pData;
    (void)size;
    return eSTATUS_SUCCESS;
}

void Gps_DataReceivedHandler (uint8_t const* pData, uint32_t size);
eSTATUS_t Gps_Read (void* ctx, bool forcePolling, GpsData_t* pOutData);
bool Gps_IsDataReady (void* ctx);

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
    /* Drain any partial sentence a previous test left in the assembler. */
    uint8_t const nl = '\n';
    Gps_DataReceivedHandler (&nl, 1U);
    GpsData_t scratch;
    (void)Gps_Read (NULL, false, &scratch);
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
    RUN_TEST (test_VoidRmc_IsNotReportedAsAFix);
    RUN_TEST (test_GgaNoFix_IsNotReportedAsAFix);
    RUN_TEST (test_NonPositionalSentence_DoesNotRepublish);
    RUN_TEST (test_BadChecksum_IsRejected);
    RUN_TEST (test_NullArgs_AreRejected);
    return UNITY_END ();
}
