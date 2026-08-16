/*
 * CRSF receiver conformance tests, written against the TBS CRSF specification
 * rather than against the current implementation.
 *
 * Spec points exercised here:
 *   - Frame layout [sync][len][type][payload][crc8], len = type + payload + crc,
 *     valid range 2..62.
 *   - CRC8 poly 0xD5 over type + payload only.
 *   - 0x16 RC Channels Packed: 16 channels x 11 bits = 22 bytes, packed LSB
 *     first, values 0..2047 (UNSIGNED - the spec's own struct sample says `int`,
 *     but signed 11-bit cannot represent the documented 1811 maximum).
 *   - TICKS_TO_US(x) = (x - 992) * 5 / 8 + 1500, i.e. 192/992/1792 ticks are
 *     1000/1500/2000 us.
 *   - "Frame size may be bigger than expected frame of given type. This should
 *     not be a reason to count the frame invalid ... just ignore extra fields."
 *
 * Not covered: the inter-frame timeout resync. GetMicroseconds() returns 0 under
 * UNIT_TEST (core_shared.c), so the timeout branch is unreachable from here.
 * It is equally unreachable in the Renode SIL, which does not pace bytes at the
 * baud rate - covering it needs a time seam that does not exist yet.
 */

#include "drivers/rx/crsf.h"
#include "drivers/rx/rx.h"

#include "unity/unity.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifndef UNIT_TEST
#error "UNIT_TEST should be defined in this file"
#endif

/* Set by the deframer once a whole frame has landed; owned by crsf.c. */
extern bool volatile s_IsFrameComplete;

/*
 * crsf.c reaches the UART through Crsf_Init and the (unused) Crsf_Bind. Nothing
 * below drives a real port - the deframer is fed directly, exactly as the RX
 * ISR feeds it - so these satisfy the linker instead of pulling in uart.c, which
 * does not build against stubs/hal_stub.c.
 */
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

#define CRSF_RC_PAYLOAD_BYTES 22U
#define CRSF_RC_FRAME_LEN     (CRSF_RC_PAYLOAD_BYTES + 2U) /* type + payload + crc = 24 */

/*
 * The spec's 22-byte packed payload. Load-bearing for this whole file: under
 * MinGW's default -mms-bitfields the same struct lays out as 32 bytes, and the
 * host would silently test a wire format the firmware never produces. The build
 * forces -mno-ms-bitfields; this catches it if that ever comes undone.
 */
_Static_assert (sizeof (CrsfChannelsPayload_t) == CRSF_RC_PAYLOAD_BYTES,
                "CrsfChannelsPayload_t must pack to 22 bytes, as it does on ARM EABI");

/* Spec channel values. */
#define TICKS_MIN  172U  /* stick low  -> 988 us  */
#define TICKS_1000 192U  /* exactly 1000 us       */
#define TICKS_MID  992U  /* exactly 1500 us       */
#define TICKS_2000 1792U /* exactly 2000 us       */
#define TICKS_MAX  1811U /* stick high -> 2011 us */

// ---------------------------------------------------------------------------
// Host-side CRSF encoder (mirrors the spec, not the firmware)
// ---------------------------------------------------------------------------

static uint8_t TestCrc8 (uint8_t const* pData, uint32_t len) {
    uint8_t crc = 0U;
    for (uint32_t i = 0; i < len; ++i) {
        crc ^= pData[i];
        for (uint8_t b = 0; b < 8U; ++b) {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0xD5U) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* 16 x 11 bits, LSB first - the layout a little-endian packed bitfield struct
 * produces, which is what crsf.c casts the payload to. */
static void PackChannels (uint32_t const ticks[RC_MAX_CHANNELS], uint8_t out[CRSF_RC_PAYLOAD_BYTES]) {
    memset (out, 0, CRSF_RC_PAYLOAD_BYTES);
    uint32_t bit = 0;
    for (uint32_t ch = 0; ch < RC_MAX_CHANNELS; ++ch) {
        uint32_t v = ticks[ch] & 0x7FFU;
        for (uint32_t b = 0; b < 11U; ++b, ++bit) {
            if (v & (1U << b)) {
                out[bit / 8U] |= (uint8_t)(1U << (bit % 8U));
            }
        }
    }
}

/* Build [0xC8][len][type][payload][crc]. Returns total byte count. */
static uint32_t BuildFrame (uint8_t type, uint8_t const* pPayload, uint8_t payloadLen, uint8_t out[CRSF_MAX_FRAME_SIZE]) {
    uint8_t const frameLen = (uint8_t)(payloadLen + 2U); /* type + payload + crc */
    out[0]                 = CRSF_SYNC_BYTE;
    out[1]                 = frameLen;
    out[2]                 = type;
    if (payloadLen) {
        memcpy (&out[3], pPayload, payloadLen);
    }
    out[3 + payloadLen] = TestCrc8 (&out[2], (uint32_t)payloadLen + 1U); /* type + payload */
    return (uint32_t)payloadLen + 4U;
}

/* Feed a frame to the deframer one byte at a time, exactly as the UART ISR does. */
static void FeedBytes (uint8_t const* pBytes, uint32_t len) {
    for (uint32_t i = 0; i < len; ++i) {
        Crsf_DataReceivedHandler_ (&pBytes[i], 1U);
    }
}

/* Build + feed an RC frame carrying `ticks`, with an optional oversized payload
 * (spec: extra trailing fields must be ignored, not rejected). */
static void FeedRcFrame (uint32_t const ticks[RC_MAX_CHANNELS], uint8_t payloadLen) {
    uint8_t payload[CRSF_MAX_FRAME_SIZE] = { 0 };
    uint8_t frame[CRSF_MAX_FRAME_SIZE]   = { 0 };

    PackChannels (ticks, payload);
    uint32_t const n = BuildFrame (CRSF_FRAME_TYPE_RC_CHANNELS, payload, payloadLen, frame);
    FeedBytes (frame, n);
}

static void FillChannels (uint32_t ticks[RC_MAX_CHANNELS], uint32_t value) {
    for (uint32_t i = 0; i < RC_MAX_CHANNELS; ++i) {
        ticks[i] = value;
    }
}

void setUp (void) {
    s_IsFrameComplete = false;
}

void tearDown (void) {
}

// ---------------------------------------------------------------------------
// Crsf_MapChannel - spec conversion
// ---------------------------------------------------------------------------

/* TICKS_TO_US(x) = (x - 992) * 5 / 8 + 1500, clamped into RC_CHANNEL_MIN/MAX. */
void test_MapChannel_SpecEndpoints (void) {
    TEST_ASSERT_EQUAL_UINT32 (1000U, Crsf_MapChannel (TICKS_1000));
    TEST_ASSERT_EQUAL_UINT32 (1500U, Crsf_MapChannel (TICKS_MID));
    TEST_ASSERT_EQUAL_UINT32 (2000U, Crsf_MapChannel (TICKS_2000));
}

/* A real transmitter swings past the 1000-2000 us window; both ends clamp. */
void test_MapChannel_ClampsTransmitterRange (void) {
    TEST_ASSERT_EQUAL_UINT32 (1000U, Crsf_MapChannel (TICKS_MIN));
    TEST_ASSERT_EQUAL_UINT32 (2000U, Crsf_MapChannel (TICKS_MAX));
    TEST_ASSERT_EQUAL_UINT32 (1000U, Crsf_MapChannel (0U));
    TEST_ASSERT_EQUAL_UINT32 (2000U, Crsf_MapChannel (2047U));
}

/* Quarter-stick points, +/-1 us for integer rounding. */
void test_MapChannel_Midpoints (void) {
    TEST_ASSERT_UINT32_WITHIN (1U, 1250U, Crsf_MapChannel (592U));  /* (592-992)*5/8+1500 */
    TEST_ASSERT_UINT32_WITHIN (1U, 1750U, Crsf_MapChannel (1392U)); /* (1392-992)*5/8+1500 */
}

/* Monotonic across the whole tick range - no wrap, no fold-back. This is what
 * a signed 11-bit channel field breaks: everything above 1023 reads negative. */
void test_MapChannel_MonotonicAcrossRange (void) {
    uint32_t prev = Crsf_MapChannel (0U);
    for (uint32_t t = 1U; t <= 2047U; ++t) {
        uint32_t const us = Crsf_MapChannel (t);
        TEST_ASSERT_TRUE_MESSAGE (us >= prev, "Crsf_MapChannel must be non-decreasing in ticks");
        TEST_ASSERT_TRUE (us >= RC_CHANNEL_MIN && us <= RC_CHANNEL_MAX);
        prev = us;
    }
}

// ---------------------------------------------------------------------------
// Deframing + decode
// ---------------------------------------------------------------------------

void test_ProcessFrame_NoFrameYet (void) {
    uint32_t ch[RC_MAX_CHANNELS] = { 0 };
    TEST_ASSERT_TRUE (STATUS_FAIL (Crsf_ProcessFrame (ch)));
}

void test_ProcessFrame_NullArg (void) {
    TEST_ASSERT_TRUE (STATUS_FAIL (Crsf_ProcessFrame (NULL)));
}

/* The everyday case: centred sticks, throttle low, AUX1 high (the arm gesture). */
void test_ProcessFrame_ValidRcFrame (void) {
    uint32_t ticks[RC_MAX_CHANNELS];
    FillChannels (ticks, TICKS_MID);
    ticks[RC_CHANNEL_IDX_THROTTLE] = TICKS_1000;
    ticks[RC_CHANNEL_IDX_AUX_1]    = TICKS_2000;

    FeedRcFrame (ticks, CRSF_RC_PAYLOAD_BYTES);
    TEST_ASSERT_TRUE (s_IsFrameComplete);

    uint32_t ch[RC_MAX_CHANNELS] = { 0 };
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Crsf_ProcessFrame (ch));

    TEST_ASSERT_EQUAL_UINT32 (1000U, ch[RC_CHANNEL_IDX_THROTTLE]);
    TEST_ASSERT_EQUAL_UINT32 (2000U, ch[RC_CHANNEL_IDX_AUX_1]);
    TEST_ASSERT_EQUAL_UINT32 (1500U, ch[RC_CHANNEL_IDX_ROLL]);
    TEST_ASSERT_EQUAL_UINT32 (1500U, ch[RC_CHANNEL_IDX_PITCH]);
    TEST_ASSERT_EQUAL_UINT32 (1500U, ch[RC_CHANNEL_IDX_YAW]);
}

/* Every channel distinct, so a packing or ordering slip cannot hide behind
 * identical values. */
void test_ProcessFrame_AllChannelsDistinct (void) {
    uint32_t ticks[RC_MAX_CHANNELS];
    for (uint32_t i = 0; i < RC_MAX_CHANNELS; ++i) {
        ticks[i] = 200U + (i * 100U); /* 200..1700, spans the signed-11-bit boundary */
    }

    FeedRcFrame (ticks, CRSF_RC_PAYLOAD_BYTES);

    uint32_t ch[RC_MAX_CHANNELS] = { 0 };
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Crsf_ProcessFrame (ch));
    for (uint32_t i = 0; i < RC_MAX_CHANNELS; ++i) {
        TEST_ASSERT_EQUAL_UINT32 (Crsf_MapChannel (ticks[i]), ch[i]);
    }
}

/* Full-scale on every channel. Fails wherever the 11-bit fields are signed:
 * 1811 reads back as -237 and saturates. */
void test_ProcessFrame_FullScaleChannels (void) {
    uint32_t ticks[RC_MAX_CHANNELS];
    FillChannels (ticks, TICKS_MAX);

    FeedRcFrame (ticks, CRSF_RC_PAYLOAD_BYTES);

    uint32_t ch[RC_MAX_CHANNELS] = { 0 };
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Crsf_ProcessFrame (ch));
    for (uint32_t i = 0; i < RC_MAX_CHANNELS; ++i) {
        TEST_ASSERT_EQUAL_UINT32 (2000U, ch[i]);
    }
}

/* Just above the signed-11-bit boundary (1023): must not fold to full scale. */
void test_ProcessFrame_AboveSignedBoundary (void) {
    uint32_t ticks[RC_MAX_CHANNELS];
    FillChannels (ticks, TICKS_MID);
    ticks[RC_CHANNEL_IDX_ROLL] = 1024U;

    FeedRcFrame (ticks, CRSF_RC_PAYLOAD_BYTES);

    uint32_t ch[RC_MAX_CHANNELS] = { 0 };
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Crsf_ProcessFrame (ch));
    /* (1024-992)*5/8+1500 = 1520 us - nowhere near RC_CHANNEL_MAX. */
    TEST_ASSERT_UINT32_WITHIN (1U, 1520U, ch[RC_CHANNEL_IDX_ROLL]);
}

// ---------------------------------------------------------------------------
// Frame validation
// ---------------------------------------------------------------------------

void test_ProcessFrame_RejectsBadCrc (void) {
    uint8_t payload[CRSF_RC_PAYLOAD_BYTES] = { 0 };
    uint8_t frame[CRSF_MAX_FRAME_SIZE]     = { 0 };
    uint32_t ticks[RC_MAX_CHANNELS];

    FillChannels (ticks, TICKS_MID);
    PackChannels (ticks, payload);
    uint32_t const n = BuildFrame (CRSF_FRAME_TYPE_RC_CHANNELS, payload, CRSF_RC_PAYLOAD_BYTES, frame);
    frame[n - 1U] ^= 0xFFU; /* corrupt the CRC */
    FeedBytes (frame, n);

    uint32_t ch[RC_MAX_CHANNELS] = { 0 };
    TEST_ASSERT_TRUE (STATUS_FAIL (Crsf_ProcessFrame (ch)));
}

/* Spec: "Valid range is between 2 and 62 ... otherwise the frame must be
 * discarded." A length byte of 1 completes at 3 bytes and must be rejected. */
void test_ProcessFrame_RejectsLengthBelowSpecMin (void) {
    uint8_t const frame[3] = { CRSF_SYNC_BYTE, 0x01U, CRSF_FRAME_TYPE_RC_CHANNELS };
    FeedBytes (frame, sizeof (frame));

    uint32_t ch[RC_MAX_CHANNELS] = { 0 };
    TEST_ASSERT_TRUE (STATUS_FAIL (Crsf_ProcessFrame (ch)));
}

/* Spec: a longer-than-expected frame is valid; ignore the extra fields.
 * 30 bytes of payload => length 32, well past the 22-byte RC payload. */
void test_ProcessFrame_AcceptsOversizedRcFrame (void) {
    uint32_t ticks[RC_MAX_CHANNELS];
    FillChannels (ticks, TICKS_MID);
    ticks[RC_CHANNEL_IDX_THROTTLE] = TICKS_1000;

    FeedRcFrame (ticks, 30U);

    uint32_t ch[RC_MAX_CHANNELS] = { 0 };
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Crsf_ProcessFrame (ch));
    TEST_ASSERT_EQUAL_UINT32 (1000U, ch[RC_CHANNEL_IDX_THROTTLE]);
    TEST_ASSERT_EQUAL_UINT32 (1500U, ch[RC_CHANNEL_IDX_ROLL]);
}

/* The largest frame the spec allows: length 62, i.e. 60 payload bytes and 64
 * bytes on the wire. Must still decode. */
void test_ProcessFrame_AcceptsMaxSpecLengthFrame (void) {
    uint32_t ticks[RC_MAX_CHANNELS];
    FillChannels (ticks, TICKS_MID);
    ticks[RC_CHANNEL_IDX_YAW] = TICKS_2000;

    FeedRcFrame (ticks, 60U);

    uint32_t ch[RC_MAX_CHANNELS] = { 0 };
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Crsf_ProcessFrame (ch));
    TEST_ASSERT_EQUAL_UINT32 (2000U, ch[RC_CHANNEL_IDX_YAW]);
}

/* A telemetry frame the FC does not handle (0x14 Link Statistics) must be
 * consumed without disturbing the deframer - the next RC frame still decodes. */
void test_ProcessFrame_UnknownTypeDoesNotCorruptState (void) {
    uint8_t payload[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    uint8_t frame[CRSF_MAX_FRAME_SIZE] = { 0 };
    uint32_t const n = BuildFrame (0x14U, payload, sizeof (payload), frame);
    FeedBytes (frame, n);

    uint32_t ch[RC_MAX_CHANNELS] = { 0 };
    TEST_ASSERT_TRUE (STATUS_FAIL (Crsf_ProcessFrame (ch)));

    uint32_t ticks[RC_MAX_CHANNELS];
    FillChannels (ticks, TICKS_MID);
    ticks[RC_CHANNEL_IDX_PITCH] = TICKS_2000;
    FeedRcFrame (ticks, CRSF_RC_PAYLOAD_BYTES);

    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Crsf_ProcessFrame (ch));
    TEST_ASSERT_EQUAL_UINT32 (2000U, ch[RC_CHANNEL_IDX_PITCH]);
}

/* Back-to-back frames with no gap: the deframer is length-driven, so the second
 * must decode cleanly. This is exactly how Renode delivers bytes (no baud
 * pacing), so it is the SIL's real arrival pattern. */
void test_ProcessFrame_BackToBackFrames (void) {
    uint32_t ticks[RC_MAX_CHANNELS];
    uint32_t ch[RC_MAX_CHANNELS] = { 0 };

    FillChannels (ticks, TICKS_MID);
    ticks[RC_CHANNEL_IDX_ROLL] = TICKS_1000;
    FeedRcFrame (ticks, CRSF_RC_PAYLOAD_BYTES);

    ticks[RC_CHANNEL_IDX_ROLL] = TICKS_2000;
    FeedRcFrame (ticks, CRSF_RC_PAYLOAD_BYTES);

    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Crsf_ProcessFrame (ch));
    TEST_ASSERT_EQUAL_UINT32 (2000U, ch[RC_CHANNEL_IDX_ROLL]);
}

/* ProcessFrame consumes the completion flag: a second call with no new frame
 * must fail rather than re-report stale channels. */
void test_ProcessFrame_ConsumesFrame (void) {
    uint32_t ticks[RC_MAX_CHANNELS];
    FillChannels (ticks, TICKS_MID);
    FeedRcFrame (ticks, CRSF_RC_PAYLOAD_BYTES);

    uint32_t ch[RC_MAX_CHANNELS] = { 0 };
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Crsf_ProcessFrame (ch));
    TEST_ASSERT_TRUE (STATUS_FAIL (Crsf_ProcessFrame (ch)));
}

/*
 * Golden frame, produced by Scripts/sim/crsf.py:
 *
 *     ch = [1500]*16; ch[3] = 1000; ch[4] = 2000
 *     crsf.rc_frame(ch)
 *
 * i.e. centred sticks, throttle low, AUX1 high - the arming gesture the SIL
 * bridge sends. Asserting the literal bytes here is what stops the host encoder
 * and this decoder from drifting apart: a bit-packing or CRC change on either
 * side breaks this test rather than silently producing garbage channels in the
 * SIL. If crsf.py changes, regenerate these bytes.
 */
void test_ProcessFrame_GoldenFrameFromHostEncoder (void) {
    static uint8_t const golden[] = {
        0xC8, 0x18, 0x16, 0xE0, 0x03, 0x1F, 0xF8, 0x80, 0x01, 0x70, 0xF0, 0x81, 0x0F,
        0x7C, 0xE0, 0x03, 0x1F, 0xF8, 0xC0, 0x07, 0x3E, 0xF0, 0x81, 0x0F, 0x7C, 0xF9
    };
    TEST_ASSERT_EQUAL_UINT32 (CRSF_RC_PAYLOAD_BYTES + 4U, sizeof (golden));

    /* The test's own encoder must agree with crsf.py byte for byte. */
    uint32_t ticks[RC_MAX_CHANNELS];
    uint8_t payload[CRSF_MAX_FRAME_SIZE] = { 0 };
    uint8_t built[CRSF_MAX_FRAME_SIZE]   = { 0 };

    FillChannels (ticks, TICKS_MID);
    ticks[RC_CHANNEL_IDX_THROTTLE] = TICKS_1000;
    ticks[RC_CHANNEL_IDX_AUX_1]    = TICKS_2000;
    PackChannels (ticks, payload);
    uint32_t const n = BuildFrame (CRSF_FRAME_TYPE_RC_CHANNELS, payload, CRSF_RC_PAYLOAD_BYTES, built);
    TEST_ASSERT_EQUAL_UINT32 (sizeof (golden), n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY (golden, built, sizeof (golden));

    /* And the firmware must decode those exact bytes to the intended sticks. */
    FeedBytes (golden, sizeof (golden));

    uint32_t ch[RC_MAX_CHANNELS] = { 0 };
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Crsf_ProcessFrame (ch));
    TEST_ASSERT_EQUAL_UINT32 (1000U, ch[RC_CHANNEL_IDX_THROTTLE]);
    TEST_ASSERT_EQUAL_UINT32 (2000U, ch[RC_CHANNEL_IDX_AUX_1]);
    for (uint32_t i = 0; i < RC_MAX_CHANNELS; ++i) {
        if (i != RC_CHANNEL_IDX_THROTTLE && i != RC_CHANNEL_IDX_AUX_1) {
            TEST_ASSERT_EQUAL_UINT32 (1500U, ch[i]);
        }
    }
}

int main (void) {
    UNITY_BEGIN ();

    RUN_TEST (test_MapChannel_SpecEndpoints);
    RUN_TEST (test_MapChannel_ClampsTransmitterRange);
    RUN_TEST (test_MapChannel_Midpoints);
    RUN_TEST (test_MapChannel_MonotonicAcrossRange);

    RUN_TEST (test_ProcessFrame_NoFrameYet);
    RUN_TEST (test_ProcessFrame_NullArg);
    RUN_TEST (test_ProcessFrame_ValidRcFrame);
    RUN_TEST (test_ProcessFrame_AllChannelsDistinct);
    RUN_TEST (test_ProcessFrame_FullScaleChannels);
    RUN_TEST (test_ProcessFrame_AboveSignedBoundary);

    RUN_TEST (test_ProcessFrame_RejectsBadCrc);
    RUN_TEST (test_ProcessFrame_RejectsLengthBelowSpecMin);
    RUN_TEST (test_ProcessFrame_AcceptsOversizedRcFrame);
    RUN_TEST (test_ProcessFrame_AcceptsMaxSpecLengthFrame);
    RUN_TEST (test_ProcessFrame_UnknownTypeDoesNotCorruptState);
    RUN_TEST (test_ProcessFrame_BackToBackFrames);
    RUN_TEST (test_ProcessFrame_ConsumesFrame);
    RUN_TEST (test_ProcessFrame_GoldenFrameFromHostEncoder);

    return UNITY_END ();
}
