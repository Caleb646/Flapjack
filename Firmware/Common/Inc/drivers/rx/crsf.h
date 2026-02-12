#ifndef DRIVERS_RX_CRSF_H
#define DRIVERS_RX_CRSF_H

#include <stdbool.h>
#include <stdint.h>

#include "core/core.h"

#include "peripheral/bus/uart.h"

#include "drivers/rx/rx.h"

#define CRSF_MAX_FRAME_SIZE            64U
#define CRSF_MAX_PAYLOAD_SIZE          58U

#define CRSF_CHANNEL_MIN               508U
#define CRSF_CHANNEL_MID               992U
#define CRSF_CHANNEL_MAX               1496U

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

// Center (1500µs) = 992, range is 992 ± 512 (508-1496)
typedef struct __attribute__ ((__packed__)) CrsfChannelsPayload_s {
    int channel_1 : 11;
    int channel_2 : 11;
    int channel_3 : 11;
    int channel_4 : 11;
    int channel_5 : 11;
    int channel_6 : 11;
    int channel_7 : 11;
    int channel_8 : 11;
    int channel_9 : 11;
    int channel_10 : 11;
    int channel_11 : 11;
    int channel_12 : 11;
    int channel_13 : 11;
    int channel_14 : 11;
    int channel_15 : 11;
    int channel_16 : 11;
} CrsfChannelsPayload_t;

eSTATUS_t Crsf_Init (UartPort_t* pPort);
eSTATUS_t Crsf_ProcessFrame (uint32_t outChannels[RC_MAX_CHANNELS]);
void Crsf_DataReceivedHandler_ (uint8_t const* pData, uint32_t len);


#endif /* DRIVERS_RX_CRSF_H */