/*
 * The tfp_format printf subset behind every LOG_* call.
 *
 * Tested through tfp_sprintf so the output can be compared directly. The
 * long-modifier cases matter on the target: arm-none-eabi makes uint32_t an
 * unsigned long, so PRIu32 expands to "lu". Without 'l' support "%lu" printed a
 * literal 'u' and left its argument on the va_list, shifting every later
 * argument into the wrong slot.
 *
 * The LOG_DATA macros are checked here too, since their whole output is a
 * format string assembled by the preprocessor.
 */

#include "unity/unity.h"

#include "common/maths.h"
#include "core/log/format.h"
#include "core/log/logger.h"

#include <limits.h>
#include <stdint.h>
#include <string.h>

static char gBuf[256];
static char* gpCapture;

static void CapturePutc (void* p, char ch) {
    (void)p;
    *gpCapture++ = ch;
    *gpCapture   = '\0';
}

static void Capture (char const* fmt, ...) __attribute__ ((format (printf, 1, 2)));
static void Capture (char const* fmt, ...) {
    va_list va;
    va_start (va, fmt);
    tfp_format (NULL, CapturePutc, fmt, va);
    va_end (va);
}

/*
 * The LOG_DATA macros expand PRINT at their use site, so pointing it at
 * Capture runs them through the real formatter instead of host printf.
 */
#undef PRINT
#define PRINT Capture

void setUp (void) {
}

void tearDown (void) {
}

static void test_Format_LongUnsigned (void) {
    tfp_sprintf (gBuf, "%lu", 4000000000UL);
    TEST_ASSERT_EQUAL_STRING ("4000000000", gBuf);
}

static void test_Format_LongSigned (void) {
    tfp_sprintf (gBuf, "%ld|%ld|%ld", -123456789L, 0L, 2147483647L);
    TEST_ASSERT_EQUAL_STRING ("-123456789|0|2147483647", gBuf);
}

static void test_Format_LongHex (void) {
    tfp_sprintf (gBuf, "%lx|%08lX", 0xdeadbeefUL, 0xABCUL);
    TEST_ASSERT_EQUAL_STRING ("deadbeef|00000ABC", gBuf);
}

static void test_Format_LongDoesNotShiftLaterArguments (void) {
    tfp_sprintf (gBuf, "%lu %s %d", 7UL, "ok", -3);
    TEST_ASSERT_EQUAL_STRING ("7 ok -3", gBuf);
}

static void test_Format_LongMaxFitsTheDigitBuffer (void) {
    tfp_sprintf (gBuf, "%lu", ULONG_MAX);
    TEST_ASSERT_EQUAL_STRING ((sizeof (long) == 8) ? "18446744073709551615" : "4294967295", gBuf);
}

static void test_Format_PlainSpecifiersUnchanged (void) {
    tfp_sprintf (gBuf, "%d %u %x %X %c %s %5d %03u %%", -42, 42U, 0xabU, 0xabU, 'z', "str", 7, 9U);
    TEST_ASSERT_EQUAL_STRING ("-42 42 ab AB z str     7 009 %", gBuf);
}

static void test_LogData_FourFloatsPrintsValuesInMilliUnits (void) {
    gpCapture = gBuf;
    LOG_4_FLOATS ("t", a, 1.5F, b, -0.25F, c, 0.0F, d, 2.0F);
    TEST_ASSERT_EQUAL_STRING ("<{\"type\":\"t\",\"core\":\"Unknown\",\"data\":{\"a\":1500,\"b\":-250,\"c\":0,\"d\":2000}}>\r\n", gBuf);
}

static void test_LogData_ImuGyroAboveInt16RangeDoesNotWrap (void) {
    Vec3f const accel = { .x = 0.0F, .y = 0.0F, .z = 9.81F };
    Vec3f const gyro  = { .x = 500.0F, .y = -1999.5F, .z = 32.768F };
    gpCapture         = gBuf;
    LOG_DATA_IMU_DATA (accel, gyro);
    TEST_ASSERT_NOT_NULL (strstr (gBuf, "{\"ax\":0,\"ay\":0,\"az\":9810,\"gx\":500000,\"gy\":-1999500,\"gz\":32768}"));
}

int main (void) {
    UNITY_BEGIN ();
    RUN_TEST (test_Format_LongUnsigned);
    RUN_TEST (test_Format_LongSigned);
    RUN_TEST (test_Format_LongHex);
    RUN_TEST (test_Format_LongDoesNotShiftLaterArguments);
    RUN_TEST (test_Format_LongMaxFitsTheDigitBuffer);
    RUN_TEST (test_Format_PlainSpecifiersUnchanged);
    RUN_TEST (test_LogData_FourFloatsPrintsValuesInMilliUnits);
    RUN_TEST (test_LogData_ImuGyroAboveInt16RangeDoesNotWrap);
    return UNITY_END ();
}
