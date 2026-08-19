/*
 * MMC5983MA magnetometer backend. Sits on SPI5, sharing the bus with the
 * barometer on its own NSS (target/flapjack_v1/flapjack_v1.h).
 *
 * The part runs in continuous measurement mode at 1000 Hz, so Read never waits
 * on it: it checks MEAS_M_DONE once and reports eSTATUS_BUSY when no new sample
 * has landed. Sampling rate is Mag_Task's to choose, not the driver's.
 *
 * Continuous mode is also why there is no one-shot TM_M trigger here. Firing
 * one per read and then waiting out the conversion - which is what this driver
 * used to do - duplicates work the part is already doing on its own clock, and
 * spends the wait hammering the SPI bus the barometer shares.
 */

#include "hal.h"
#include "target.h"

#include "core/core.h"

#include "drivers/bus/spi.h"

#include "drivers/mag/magdrv.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#if defined(MAG_SPI_BUS_ID)

#define MMC5983_PROD_ID        0x30U

#define MMC5983_REG_X_OUT_0    0x00U
#define MMC5983_REG_STATUS     0x08U
#define MMC5983_REG_INT_CTRL_0 0x09U
#define MMC5983_REG_INT_CTRL_1 0x0AU
#define MMC5983_REG_INT_CTRL_2 0x0BU
#define MMC5983_REG_PROD_ID    0x2FU

// STATUS
#define MMC5983_MEAS_M_DONE    (1U << 0U)

// INT_CTRL_0
#define MMC5983_AUTO_SR_EN     (1U << 5U)

// INT_CTRL_1
#define MMC5983_BW0            (1U << 0U)
#define MMC5983_BW1            (1U << 1U)
#define MMC5983_SW_RST         (1U << 7U)

// INT_CTRL_2
#define MMC5983_CM_FREQ_0      (1U << 0U)
#define MMC5983_CM_FREQ_1      (1U << 1U)
#define MMC5983_CM_FREQ_2      (1U << 2U)
#define MMC5983_CMM_EN         (1U << 3U)
#define MMC5983_PRD_SET_0      (1U << 4U)
#define MMC5983_PRD_SET_1      (1U << 5U)
#define MMC5983_PRD_SET_2      (1U << 6U)
#define MMC5983_EN_PRD_SET     (1U << 7U)

#define DATA_LEN               7U // X/Y/Z high+low bytes, then their shared low-2-bit byte

// Output is 18-bit unsigned with zero field at mid-scale, so the midpoint is
// both the zero offset and the full-scale divisor.
#define MAG_SIGNED_POS_MAX     (1U << 17U)

#define RESET_SETTLE_US        15000U // 10 ms reset time + 5 ms margin

typedef struct Mmc5983_s {
    SpiDev_t spiDev;
    bool normalize;
} Mmc5983_t;

STATIC eSTATUS_t MagRead (Mmc5983_t* pMag, uint8_t reg, uint8_t* pData, uint16_t size) {

    eSTATUS_t status = SpiDev_ReadRegister (&pMag->spiDev, reg, pData, size);
    DelayMicroseconds (2);
    return status;
}

STATIC eSTATUS_t MagWrite (Mmc5983_t* pMag, uint8_t reg, uint8_t value) {

    eSTATUS_t status = SpiDev_WriteRegister (&pMag->spiDev, reg, &value, 1U);
    DelayMicroseconds (2);
    return status;
}

STATIC eSTATUS_t MagSoftReset (Mmc5983_t* pMag) {

    eSTATUS_t status = MagWrite (pMag, MMC5983_REG_INT_CTRL_1, MMC5983_SW_RST);
    DelayMicroseconds (RESET_SETTLE_US);
    return status;
}

STATIC Vec3f MagRaw2Norm (Mmc5983_t const* pMag, Vec3u raw) {

    Vec3f output = { 0 };
    output.x     = ((float)raw.x) - (float)MAG_SIGNED_POS_MAX;
    output.y     = ((float)raw.y) - (float)MAG_SIGNED_POS_MAX;
    output.z     = ((float)raw.z) - (float)MAG_SIGNED_POS_MAX;
    if (pMag->normalize == true) {
        output.x /= (float)MAG_SIGNED_POS_MAX;
        output.y /= (float)MAG_SIGNED_POS_MAX;
        output.z /= (float)MAG_SIGNED_POS_MAX;
    }
    return output;
}

STATIC bool Mmc5983_IsDataReady (void* ctx) {

    Mmc5983_t* pMag = (Mmc5983_t*)ctx;
    if (!pMag) {
        return false;
    }

    uint8_t status = 0;
    if (MagRead (pMag, MMC5983_REG_STATUS, &status, 1U) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to read MAG status register");
        return false;
    }
    return (status & MMC5983_MEAS_M_DONE) > 0U;
}

STATIC eSTATUS_t Mmc5983_HwInit (Mmc5983_t* pMag) {

    pMag->spiDev.cfg.busId    = MAG_SPI_BUS_ID;
    pMag->spiDev.cfg.pNssPort = MAG_SPI_NSS_GPIO_PORT;
    pMag->spiDev.cfg.nssPin   = MAG_SPI_NSS_GPIO_PIN;
    if (SpiDev_Init (&pMag->spiDev) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize SPI device for MAG");
        return eSTATUS_FAILURE;
    }

    GOTO_IF (MagSoftReset (pMag) != eSTATUS_SUCCESS, error, "Failed to soft reset MAG");

    uint8_t chipId = 0;
    GOTO_IF (MagRead (pMag, MMC5983_REG_PROD_ID, &chipId, 1U) != eSTATUS_SUCCESS, error, "Failed to read MAG chip ID");
    GOTO_IF (chipId != MMC5983_PROD_ID, error, "Invalid MAG chip ID");

    // Automatic set/reset, which cancels the bridge offset the part accumulates.
    GOTO_IF (MagWrite (pMag, MMC5983_REG_INT_CTRL_0, MMC5983_AUTO_SR_EN) != eSTATUS_SUCCESS, error, "Failed to configure MAG INT_CTRL_0 register");

    // 800 Hz bandwidth: the shortest measurement time, which is what the 1000 Hz
    // continuous rate set below needs to keep up with.
    GOTO_IF (MagWrite (pMag, MMC5983_REG_INT_CTRL_1, MMC5983_BW0 | MMC5983_BW1) != eSTATUS_SUCCESS, error, "Failed to configure MAG INT_CTRL_1 register");

    // Last: this starts continuous conversions, so bandwidth and set/reset above
    // have to already be in place. PRD_SET picks how often the automatic
    // set/reset runs.
    uint8_t ctrl2 = MMC5983_CM_FREQ_0 | MMC5983_CM_FREQ_1 | MMC5983_CM_FREQ_2 | MMC5983_CMM_EN;
    ctrl2 |= MMC5983_PRD_SET_0 | MMC5983_PRD_SET_1 | MMC5983_PRD_SET_2 | MMC5983_EN_PRD_SET;
    GOTO_IF (MagWrite (pMag, MMC5983_REG_INT_CTRL_2, ctrl2) != eSTATUS_SUCCESS, error, "Failed to configure MAG INT_CTRL_2 register");

    LOG_INFO ("Successfully initialized MAG");
    return eSTATUS_SUCCESS;

error:
    memset (pMag, 0, sizeof (Mmc5983_t));
    return eSTATUS_FAILURE;
}

STATIC eSTATUS_t Mmc5983_Read (void* ctx, bool forcePolling, MagData_t* pOutData) {

    FJ_UNUSED (forcePolling);
    Mmc5983_t* pMag = (Mmc5983_t*)ctx;
    if (!pMag || !pOutData) {
        return eSTATUS_NULL_ARG;
    }

    // One drdy check, no wait. Reporting BUSY rather than re-reading stops a
    // sample being published twice, the same contract bmp390.c keeps.
    if (Mmc5983_IsDataReady (pMag) == false) {
        return eSTATUS_BUSY;
    }

    // One burst: the low two bits of all three axes share the seventh byte, so
    // reading it separately can pair them with a different measurement.
    uint8_t raw[DATA_LEN] = { 0 };
    eSTATUS_t status      = MagRead (pMag, MMC5983_REG_X_OUT_0, raw, DATA_LEN);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to read MAG data registers");
        return status;
    }

    uint32_t x1 = raw[0], x2 = raw[1];
    uint32_t y1 = raw[2], y2 = raw[3];
    uint32_t z1 = raw[4], z2 = raw[5];
    uint32_t xyz = raw[6];

    Vec3u rawField = { 0 };
    rawField.x     = (((x1 << 8U) | x2) << 2U) | (xyz >> 6U);
    rawField.y     = (((y1 << 8U) | y2) << 2U) | ((xyz >> 4U) & 0x3U);
    rawField.z     = (((z1 << 8U) | z2) << 2U) | ((xyz >> 2U) & 0x3U);

    pOutData->field = MagRaw2Norm (pMag, rawField);
    return eSTATUS_SUCCESS;
}

eSTATUS_t MagDrv_Init (MagDriverConf_t const* pConf, MagDriver_t* pOutDriver) {

    if (!pConf || !pOutDriver) {
        return eSTATUS_NULL_ARG;
    }

    memset (pOutDriver, 0, sizeof (MagDriver_t));
    pOutDriver->ctx = Allocate (sizeof (Mmc5983_t));
    if (!pOutDriver->ctx) {
        return eSTATUS_FAILURE;
    }
    pOutDriver->Read        = Mmc5983_Read;
    pOutDriver->IsDataReady = Mmc5983_IsDataReady;

    Mmc5983_t* pMag = (Mmc5983_t*)pOutDriver->ctx;
    memset (pMag, 0, sizeof (Mmc5983_t));
    pMag->normalize = pConf->normalize;

    return Mmc5983_HwInit (pMag);
}

#else // board declares no magnetometer wiring

eSTATUS_t MagDrv_Init (MagDriverConf_t const* pConf, MagDriver_t* pOutDriver) {

    FJ_UNUSED (pConf);
    FJ_UNUSED (pOutDriver);
    return eSTATUS_UNSUPPORTED;
}

#endif // MAG_SPI_BUS_ID
