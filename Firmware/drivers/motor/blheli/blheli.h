#ifndef DRIVERS_MOTOR_BLHELI_BLHELI_H
#define DRIVERS_MOTOR_BLHELI_BLHELI_H

/*
 * BLHeli_S settings, written through the ESC's own serial bootloader on the
 * motor signal wire - the same path BLHeliSuite and BLHeli Configurator use,
 * minus the PC. Private to the DShot motor backend.
 *
 * blheli.c holds the pure half (CRC, settings merge) so the host tests can
 * reach it; blheli_boot.c holds the bit-banged wire protocol.
 */

#include "hal.h"

#include "core/core.h"

#include <stdbool.h>
#include <stdint.h>

/* BLHeli_S keeps its settings in one SiLabs flash page below the bootloader
 * (0x1C00), not in a separate EEPROM. */
#define BLHELI_SETTINGS_ADDR       0x1A00U
#define BLHELI_SETTINGS_SIZE       0x70U

#define BLHELI_OFS_LAYOUT_REVISION 0x02U
#define BLHELI_OFS_STARTUP_POWER   0x09U
#define BLHELI_OFS_MOTOR_DIRECTION 0x0BU
#define BLHELI_OFS_COMM_TIMING     0x15U
#define BLHELI_OFS_DEMAG_COMP      0x1FU
#define BLHELI_OFS_TEMP_PROTECTION 0x23U
#define BLHELI_OFS_LOW_RPM_PROTECT 0x24U
#define BLHELI_OFS_BRAKE_ON_STOP   0x27U

/* BLHeli_S 16.0-16.2 and 16.3+. They differ only in TEMPERATURE_PROTECTION,
 * a bool in 32 and a 0-7 threshold index in 33. */
#define BLHELI_S_LAYOUT_32         32U
#define BLHELI_S_LAYOUT_33         33U

/* Raw EEPROM encodings, as BLHeli Configurator writes them - not the labels it
 * shows. temperatureProtection uses the layout-33 encoding. */
typedef struct {
    uint8_t motorDirection;
    uint8_t startupPower;
    uint8_t commutationTiming;
    uint8_t demagCompensation;
    uint8_t temperatureProtection;
    uint8_t lowRpmPowerProtection;
    uint8_t brakeOnStop;
} BLHeliSettings_t;

uint16_t BLHeli_Crc16 (uint16_t crc, uint8_t const* pData, uint32_t len);

/*
 * Overwrites only the fields in pWanted, leaving every other byte as the ESC
 * holds it (firmware revision, MCU and name strings, PPM range, beeps).
 * Returns eSTATUS_UNSUPPORTED and leaves the buffer untouched unless the
 * layout is BLHeli_S 32 or 33.
 */
eSTATUS_t BLHeli_MergeSettings (uint8_t settings[BLHELI_SETTINGS_SIZE], BLHeliSettings_t const* pWanted, bool* pOutChanged);

/*
 * Reads each ESC's settings and rewrites them only where they differ from
 * pWanted[i]. Must run before the scheduler starts: the bit timing is
 * busy-waited, and each line is held high long enough to drop a disarmed ESC
 * into its bootloader. Every line is left driven low, so the ESCs are back in
 * their application waiting for a DShot signal.
 */
eSTATUS_t BLHeli_ApplySettings (GPIO_TypeDef* const pPorts[], uint32_t const pins[], BLHeliSettings_t const pWanted[], uint32_t count);

#endif // DRIVERS_MOTOR_BLHELI_BLHELI_H
