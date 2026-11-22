#include "drivers/motor/blheli/blheli.h"

#include "core/core.h"

#include <stdbool.h>
#include <stdint.h>

/* CRC-16/ARC (reflected 0x8005 = 0xA001, init 0): the bootloader's own CRC, not
 * the XMODEM CRC the 4-way PC protocol wraps around it. */
uint16_t BLHeli_Crc16 (uint16_t crc, uint8_t const* pData, uint32_t len) {

    for (uint32_t i = 0; i < len; ++i) {
        uint8_t b = pData[i];
        for (uint8_t bit = 0; bit < 8U; ++bit) {
            if (((b ^ crc) & 0x01U) != 0U) {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            } else {
                crc = (uint16_t)(crc >> 1U);
            }
            b >>= 1U;
        }
    }
    return crc;
}

static void SetByte (uint8_t settings[BLHELI_SETTINGS_SIZE], uint32_t offset, uint8_t value, bool* pChanged) {

    if (settings[offset] != value) {
        settings[offset] = value;
        *pChanged        = true;
    }
}

eSTATUS_t BLHeli_MergeSettings (uint8_t settings[BLHELI_SETTINGS_SIZE], BLHeliSettings_t const* pWanted, bool* pOutChanged) {

    uint8_t const layout = settings[BLHELI_OFS_LAYOUT_REVISION];
    if (layout != BLHELI_S_LAYOUT_32 && layout != BLHELI_S_LAYOUT_33) {
        return eSTATUS_UNSUPPORTED;
    }

    uint8_t const temp = (layout == BLHELI_S_LAYOUT_32) ? (pWanted->temperatureProtection != 0U ? 1U : 0U)
                                                        : pWanted->temperatureProtection;

    bool changed = false;
    SetByte (settings, BLHELI_OFS_MOTOR_DIRECTION, pWanted->motorDirection, &changed);
    SetByte (settings, BLHELI_OFS_STARTUP_POWER, pWanted->startupPower, &changed);
    SetByte (settings, BLHELI_OFS_COMM_TIMING, pWanted->commutationTiming, &changed);
    SetByte (settings, BLHELI_OFS_DEMAG_COMP, pWanted->demagCompensation, &changed);
    SetByte (settings, BLHELI_OFS_TEMP_PROTECTION, temp, &changed);
    SetByte (settings, BLHELI_OFS_LOW_RPM_PROTECT, pWanted->lowRpmPowerProtection, &changed);
    SetByte (settings, BLHELI_OFS_BRAKE_ON_STOP, pWanted->brakeOnStop, &changed);

    *pOutChanged = changed;
    return eSTATUS_SUCCESS;
}
