#include <stdbool.h>
#include <stdint.h>

#include "core/core.h"

#include "peripheral/bus/uart.h"

#include "drivers/rx/crsf.h"
#include "drivers/rx/rx.h"

FJ_DEFINE_SHARED (CrsfFrame_t, s_CrsfFrame);
FJ_DEFINE_SHARED (UartPort_t*, s_pCrsfPort);
FJ_DEFINE_SHARED (bool volatile, s_IsFrameComplete) = false;

static uint8_t crc8 (uint8_t const* ptr, uint8_t len);

// Source: https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf.c#L685
eSTATUS_t Crsf_Bind (void) {

    uint8_t bindFrame[] = {
        CRSF_SYNC_BYTE,
        0x07, // frame length
        CRSF_FRAME_TYPE_COMMAND,
        CRSF_ADDRESS_CRSF_RECEIVER,
        CRSF_ADDRESS_FLIGHT_CONTROLLER,
        CRSF_COMMAND_SUBCMD_RX,
        CRSF_COMMAND_SUBCMD_RX_BIND,
        0x9E, // Command CRC8
        0xE8, // Packet CRC8
    };
    return UartPort_Write (s_pCrsfPort, bindFrame, sizeof (bindFrame));
}

eSTATUS_t Crsf_Init (UartPort_t* pPort) {

    if (!pPort) {
        return eSTATUS_FAILURE;
    }

    pPort->cfg.rxCallback = Crsf_DataReceivedHandler_;
    if (STATUS_FAIL (UartPort_Init (pPort))) {
        LOG_ERROR ("Failed to initialize RX UART port");
        return eSTATUS_FAILURE;
    }
    s_pCrsfPort = pPort;
    return eSTATUS_SUCCESS;
}

// source: https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md
eSTATUS_t Crsf_ProcessFrame (uint32_t outChannels[RC_MAX_CHANNELS]) {

    if (!outChannels || !s_IsFrameComplete) {
        return eSTATUS_FAILURE;
    }

    CrsfFrame_t const* pFrame = &s_CrsfFrame;
    s_IsFrameComplete         = false;

    uint8_t const frameLen = pFrame->header.length;
    if (frameLen < 2 || frameLen > CRSF_MAX_PAYLOAD_SIZE) {
        return eSTATUS_FAILURE;
    }

    uint8_t const crc = crc8 (&pFrame->header.type, frameLen - 1);
    if (crc != pFrame->bytes[frameLen + 1]) {
        return eSTATUS_FAILURE;
    }

    switch (pFrame->header.type) {
    case CRSF_FRAME_TYPE_RC_CHANNELS:
        if (frameLen != sizeof (CrsfChannelsPayload_t)) {
            return eSTATUS_FAILURE;
        }
        CrsfChannelsPayload_t* pPayload = (CrsfChannelsPayload_t*)&pFrame->bytes[3];
        outChannels[0]                  = pPayload->channel_1;
        outChannels[1]                  = pPayload->channel_2;
        outChannels[2]                  = pPayload->channel_3;
        outChannels[3]                  = pPayload->channel_4;
        outChannels[4]                  = pPayload->channel_5;
        outChannels[5]                  = pPayload->channel_6;
        outChannels[6]                  = pPayload->channel_7;
        outChannels[7]                  = pPayload->channel_8;
        outChannels[8]                  = pPayload->channel_9;
        outChannels[9]                  = pPayload->channel_10;
        outChannels[10]                 = pPayload->channel_11;
        outChannels[11]                 = pPayload->channel_12;
        outChannels[12]                 = pPayload->channel_13;
        outChannels[13]                 = pPayload->channel_14;
        outChannels[14]                 = pPayload->channel_15;
        outChannels[15]                 = pPayload->channel_16;
        break;
    default: return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

void Crsf_DataReceivedHandler_ (uint8_t const* pData, uint32_t len) {

    static uint8_t frameByteIdx          = 0;
    static uint32_t usFrameStartTime     = 0;
    static uint32_t const usFrameTimeout = (1000000 / 416666) * (CRSF_MAX_FRAME_SIZE * 8) * 2;

    if (!pData || !len) {
        return;
    }

    if (!frameByteIdx) {
        usFrameStartTime = GetMicroseconds ();
    } else if ((GetMicroseconds () - usFrameStartTime) > usFrameTimeout || frameByteIdx >= CRSF_MAX_FRAME_SIZE) {
        // if time since start of frame exceeds timeout,
        // reset frame byte index to start of new frame
        frameByteIdx     = 0;
        usFrameStartTime = GetMicroseconds ();
    }

    s_CrsfFrame.bytes[frameByteIdx++] = *pData;
    if (frameByteIdx >= 3) {
        if (frameByteIdx == s_CrsfFrame.header.length + 2) {
            frameByteIdx      = 0;
            usFrameStartTime  = 0;
            s_IsFrameComplete = true;
        }
    }
}


FJ_DEFINE_SHARED (uint8_t, crc8tab[256]) = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

/*
 * \brief CRC includes Type and Payload of each frame (doesn't include sync byte and frame length)
 */
static uint8_t crc8 (uint8_t const* ptr, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc = crc8tab[crc ^ *ptr++];
    }
    return crc;
}
