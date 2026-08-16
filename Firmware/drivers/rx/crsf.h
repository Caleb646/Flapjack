#ifndef DRIVERS_RX_CRSF_H
#define DRIVERS_RX_CRSF_H

#include <stdbool.h>
#include <stdint.h>

#include "core/core.h"

#include "drivers/serial/uart.h"

#include "drivers/rx/rx.h"

#define CRSF_MAX_FRAME_SIZE            64U
// Largest payload an extended-header frame can carry: 64 - sync - length - type
// - destination - origin - crc.
#define CRSF_MAX_PAYLOAD_SIZE          58U
// Spec: the length byte is "Type + Payload + CRC", valid range 2 to 62; a frame
// whose length falls outside that must be discarded. Note this is a bound on the
// LENGTH FIELD, not on the payload - CRSF_MAX_PAYLOAD_SIZE is the wrong constant
// to check it against.
#define CRSF_MAX_FRAME_LENGTH          62U
#define CRSF_MIN_FRAME_LENGTH          2U

/*
 * Channel values that map to RC_CHANNEL_MIN/MID/MAX (1000/1500/2000 us) under
 * the spec's conversion, TICKS_TO_US(x) = (x - 992) * 5 / 8 + 1500.
 *
 * A transmitter actually swings wider than this - roughly 172 to 1811, i.e. 988
 * to 2011 us - so Crsf_MapChannel clamps the last ~12 us at each end.
 *
 * These were 508 and 1496, which under the spec's formula are 1198 us and
 * 1815 us: a 617 us slice of stick travel stretched over the full output range,
 * with everything outside it clamped. Centre came out at 1489 us rather than
 * 1500, which guidance reads as a standing -3.96 deg/s rate demand on roll,
 * pitch and yaw with the sticks centred.
 */
#define CRSF_CHANNEL_MIN               192U
#define CRSF_CHANNEL_MID               992U
#define CRSF_CHANNEL_MAX               1792U

#define CRSF_SYNC_BYTE                 0XC8

#define CRSF_FRAME_TYPE_RC_CHANNELS    0x16U
#define CRSF_FRAME_TYPE_COMMAND        0x32U

#define CRSF_ADDRESS_CRSF_RECEIVER     0xECU
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8U

// https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md#0x320x10-crossfire
#define CRSF_COMMAND_SUBCMD_RX         0x10U
#define CRSF_COMMAND_SUBCMD_RX_BIND    0x01U

typedef uint8_t CrsfFrameType_t;

typedef struct CrsfFrame_s {
    union {
        struct {
            uint8_t firstByte;
            uint8_t length; // Valid range is between 2 and 62
            CrsfFrameType_t type;
        } header;
        uint8_t bytes[CRSF_MAX_FRAME_SIZE];
    };
} CrsfFrame_t;

/*
 * 0x16 RC Channels Packed: 16 channels x 11 bits = 22 bytes, LSB first.
 *
 * UNSIGNED deliberately, and NOT what the spec text shows. The spec samples this
 * struct with plain `int` bitfields, but signed 11-bit tops out at 1023 while the
 * spec documents channel values up to 1811 - so the sample cannot represent its
 * own range. Signed here made every value above 1023 read back negative, wrap
 * through Crsf_MapChannel's uint32_t parameter and clamp to RC_CHANNEL_MAX, so
 * the top ~40% of every stick snapped to full deflection.
 */
typedef struct __attribute__ ((__packed__)) CrsfChannelsPayload_s {
    unsigned int channel_1 : 11;
    unsigned int channel_2 : 11;
    unsigned int channel_3 : 11;
    unsigned int channel_4 : 11;
    unsigned int channel_5 : 11;
    unsigned int channel_6 : 11;
    unsigned int channel_7 : 11;
    unsigned int channel_8 : 11;
    unsigned int channel_9 : 11;
    unsigned int channel_10 : 11;
    unsigned int channel_11 : 11;
    unsigned int channel_12 : 11;
    unsigned int channel_13 : 11;
    unsigned int channel_14 : 11;
    unsigned int channel_15 : 11;
    unsigned int channel_16 : 11;
} CrsfChannelsPayload_t;

eSTATUS_t Crsf_Init (UartPort_t* pPort);
eSTATUS_t Crsf_ProcessFrame (uint32_t outChannels[RC_MAX_CHANNELS]);
void Crsf_DataReceivedHandler_ (uint8_t const* pData, uint32_t len);
// Channel value (11-bit CRSF ticks) -> pulse width in us, clamped to
// RC_CHANNEL_MIN/MAX. Exposed for unit testing.
uint32_t Crsf_MapChannel (uint32_t crsfChannelVal);


#endif /* DRIVERS_RX_CRSF_H */