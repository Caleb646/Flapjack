/*
 * BMP390 barometer backend - NOT YET IMPLEMENTED.
 *
 * This is still the null stub: BaroDrv_Init fails, so Baro_Task logs once and
 * deletes itself rather than spinning on a Read that can never succeed, the same
 * way Imu_Task and Mag_Task handle an absent part. The hardware build therefore
 * links and boots with no barometer; only the SIL (drivers/baro/sim.c) produces
 * pressure today.
 *
 * To implement, follow drivers/mag/mmc5983.c - it is the closest sibling, on the
 * same SPI bus:
 *
 *   - Board wiring is already declared: BARO_SPI_BUS_ID (eSPI_5_BUS_ID),
 *     BARO_SPI_NSS_GPIO_PORT (GPIOF), BARO_SPI_NSS_GPIO_PIN (GPIO_PIN_6) in
 *     target/flapjack_v1/flapjack_v1.h. SPI5 is shared with the magnetometer,
 *     which sits on its own NSS (pin 4).
 *   - Wrap the whole board-dependent body in `#if defined(BARO_SPI_BUS_ID)`, as
 *     mmc5983.c and bmi323.c do. nucleo_h747zi.h declares no BARO section, so an
 *     unguarded reference stops that board building at all.
 *   - Allocate the private context with Allocate() and bind it to
 *     BaroDriver_t.ctx, matching mmc5983.c / bmi323.c.
 *   - The part needs its NVM calibration coefficients read once at init and the
 *     compensation polynomial applied per sample; `Read` must return pressure
 *     already in pascals and temperature in degrees C.
 */

#include "drivers/baro/barodrv.h"

#include "core/core.h"

#include <stdbool.h>

eSTATUS_t BaroDrv_Init (BaroDriverConf_t const* pConf, BaroDriver_t* pOutDriver) {

    FJ_UNUSED (pConf);
    FJ_UNUSED (pOutDriver);
    return eSTATUS_UNSUPPORTED;
}
