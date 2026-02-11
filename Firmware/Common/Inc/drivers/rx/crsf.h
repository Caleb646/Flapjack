#ifndef DRIVERS_RX_CRSF_H
#define DRIVERS_RX_CRSF_H

#define CRSF_MAX_FRAME_SIZE            64U
#define CRSF_MAX_PAYLOAD_SIZE          58U

#define CRSF_SYNC_BYTE                 0XC8

#define CRSF_FRAME_TYPE_RC_CHANNELS    0x16U
#define CRSF_FRAMETYPE_COMMAND         0x32U

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
    int channel_01 : 11;
    int channel_02 : 11;
    int channel_03 : 11;
    int channel_04 : 11;
    int channel_05 : 11;
    int channel_06 : 11;
    int channel_07 : 11;
    int channel_08 : 11;
    int channel_09 : 11;
    int channel_10 : 11;
    int channel_11 : 11;
    int channel_12 : 11;
    int channel_13 : 11;
    int channel_14 : 11;
    int channel_15 : 11;
    int channel_16 : 11;
} CrsfChannelsPayload_t;


#endif /* DRIVERS_RX_CRSF_H */