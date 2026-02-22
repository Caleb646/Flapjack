#include "hal.h"
#include "target.h"

#include "core/core.h"

#include "drivers/sensors/mag/mag.h"
#include "drivers/sensors/mag/mmc5983.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define NUM_CONTROL_REGISTERS 4U
#define CONTROL_REG2IDX(REG) ((REG) - MMC5983_INT_CTRL_0_REG)
#define MAG_UNSIGNED_MAX     ((1U << 18U) - 1U)
#define MAG_SIGNED_POS_MAX   (1U << 17U)
#define MAG_VALID(pMAG)         ((pMAG) && (pMAG)->isInitialized)

#if BRD_IS_ENABLED(MAG)
#endif

FJ_DEFINE_SHARED (Mag_t, g_Mag)                                    = { 0 };
FJ_DEFINE_SHARED (uint8_t, s_ControlRegisters[NUM_CONTROL_REGISTERS]) = { 0 };

STATIC eSTATUS_t MagRead (Mag_t* pMag, uint8_t reg, uint8_t* pData, uint16_t size) {

    eSTATUS_t status = SpiDev_ReadRegister (&pMag->spiDev, reg, pData, size);
    DelayMicroseconds (2);
    return status;
}

STATIC eSTATUS_t MagWrite (Mag_t* pMag, uint8_t reg, uint8_t const* pData, uint16_t size) {

    eSTATUS_t status = SpiDev_WriteRegister (&pMag->spiDev, reg, pData, size);
    DelayMicroseconds (2);
    return status;
}

STATIC bool MagControlRegWrite (Mag_t* pMag, uint8_t reg, uint8_t bitMask, bool doWrite) {

    uint8_t* pValue = &s_ControlRegisters[CONTROL_REG2IDX (reg)];
    *pValue |= bitMask;
    if (doWrite) {
        if (MagWrite (pMag, reg, pValue, 1U) != eSTATUS_SUCCESS) {
            *pValue &= ~(uint32_t)bitMask; // unset the bit on failure
            LOG_ERROR ("Failed to write MAG control register");
            return false;
        }
    }
    return true;
}

STATIC uint8_t MagReadStatusReg (Mag_t* pMag) {

    uint8_t status = 0;
    if (MagRead (pMag, MMC5983_STATUS_REG, &status, 1U) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to read MAG status register");
        return 0;
    }
    return status;
}

STATIC bool MagXYZIsReady (Mag_t* pMag) {

    return (MagReadStatusReg (pMag) & MMC5983_MEAS_M_DONE) > 0 ? true : false;
}

STATIC eSTATUS_t MagSoftReset (Mag_t* pMag) {

    // eMAG_STATUS_t status = eSTATUS_SUCCESS;
    memset (s_ControlRegisters, 0, sizeof (s_ControlRegisters));

    bool success = MagControlRegWrite (pMag, MMC5983_INT_CTRL_1_REG, MMC5983_SW_RST, true);
    // 10 ms reset time + 5 ms margin
    DelayMicroseconds (15000);
    // Clear the software reset bit on our registers
    MagControlRegWrite (pMag, MMC5983_INT_CTRL_1_REG, (uint8_t)~MMC5983_SW_RST, false);
    return success == true ? eSTATUS_SUCCESS : eSTATUS_FAILURE;
}

/*
 * Could be called inside an interrupt
 */
STATIC bool MagUpdateRawData (Mag_t* pMag) {

    uint8_t pXYZ[7]  = { 0 };
    eSTATUS_t status = MagRead (pMag, MMC5983_X_OUT_0_REG, pXYZ, 7U);
    if (status != eSTATUS_SUCCESS) {
        return false;
    }

    uint32_t x1 = 0, x2 = 0, y1 = 0, y2 = 0, z1 = 0, z2 = 0, xyz = 0;
    x1 = pXYZ[0], x2 = pXYZ[1];
    y1 = pXYZ[2], y2 = pXYZ[3];
    z1 = pXYZ[4], z2 = pXYZ[5];
    xyz                    = pXYZ[6];
    pMag->rawData.x        = (((x1 << 8U) | x2) << 2U) | (xyz >> 6U);
    pMag->rawData.y        = (((y1 << 8U) | y2) << 2U) | ((xyz >> 4U) & 0x3U);
    pMag->rawData.z        = (((z1 << 8U) | z2) << 2U) | ((xyz >> 2U) & 0x3U);
    pMag->dataUpdated      = true;
    pMag->usLastUpdateTime = GetMicroseconds ();
    return true;
}

STATIC Vec3f MagRaw2NormedGauss (Mag_t const* pMag, Vec3u raw, bool doNormalization) {

    FJ_UNUSED (pMag);
    Vec3f output = { 0 };
    output.x     = ((float)raw.x) - (float)MAG_SIGNED_POS_MAX;
    output.y     = ((float)raw.y) - (float)MAG_SIGNED_POS_MAX;
    output.z     = ((float)raw.z) - (float)MAG_SIGNED_POS_MAX;
    if (doNormalization == true) {
        output.x /= (float)MAG_SIGNED_POS_MAX;
        output.y /= (float)MAG_SIGNED_POS_MAX;
        output.z /= (float)MAG_SIGNED_POS_MAX;
    }
    return output;
}

/*
 * Called by interrupt handler
 */
STATIC UNUSED_FN_DECL bool MagUpdateFromINT (Mag_t* pMag) {

    if (MagXYZIsReady (pMag) == false) {
        return false;
    }
    if (MagUpdateRawData (pMag) == false) {
        return false;
    }
    return true;
}

STATIC bool MagUpdateFromPolling (Mag_t* pMag) {

    // eMAG_STATUS_t status = eSTATUS_SUCCESS;
    bool success = true;
    // Initiate measurement
    success = MagControlRegWrite (pMag, MMC5983_INT_CTRL_0_REG, MMC5983_TM_M, true);
    GOTO_IF (success == false, error, "Failed to initiate MAG measurement");

    uint16_t timeout = 500U;
    while (MagXYZIsReady (pMag) == false && timeout-- > 0U) {
        DelayMicroseconds (5);
    }
    GOTO_IF (timeout == 0U, error, "MAG measurement timed out");

    success = MagUpdateRawData (pMag);
    GOTO_IF (success == false, error, "Failed to update MAG raw data");
    // Clear measurement done flag in saved registers
    MagControlRegWrite (pMag, MMC5983_INT_CTRL_0_REG, (uint8_t)~MMC5983_MEAS_M_DONE, false);
    return success;
error:
    MagControlRegWrite (pMag, MMC5983_INT_CTRL_0_REG, (uint8_t)~MMC5983_MEAS_M_DONE, false);
    return false;
}

eSTATUS_t Mag_Init_ (Mag_t* pOutMag) {

    eSTATUS_t status = eSTATUS_SUCCESS;
    if (SpiDev_Init (&pOutMag->spiDev) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize SPI device for MAG");
        return eSTATUS_FAILURE;
    }

    Mag_t* pMag = pOutMag;
    GOTO_IF (MagSoftReset (pMag) != eSTATUS_SUCCESS, error, "Failed to soft reset MAG");

    uint8_t chipId = 0;
    GOTO_IF (MagRead (pMag, MMC5983_PROD_ID_REG, &chipId, 1U) != eSTATUS_SUCCESS, error, "Failed to read MAG chip ID");
    GOTO_IF (chipId != MMC5983_PROD_ID, error, "Invalid MAG chip ID");

    bool success  = true;
    uint8_t flags = 0;
    flags |= MMC5983_AUTO_SR_EN; // Enable automatic set/reset
    // flags |= MMC5983_INT_MEAS_DONE_EN; // Enable measurement done interrupt
    success = MagControlRegWrite (pMag, MMC5983_INT_CTRL_0_REG, flags, true);
    GOTO_IF (success == false, error, "Failed to configure MAG INT_CTRL_0 register");

    flags   = MMC5983_BW0 | MMC5983_BW1; // Set bandwidth to 800Hz
    success = MagControlRegWrite (pMag, MMC5983_INT_CTRL_1_REG, flags, true);
    GOTO_IF (success == false, error, "Failed to configure MAG INT_CTRL_1 register");

    // Set continuous measurement mode with 1000Hz output rate
    flags = MMC5983_CM_FREQ_0 | MMC5983_CM_FREQ_1 | MMC5983_CM_FREQ_2 | MMC5983_CMM_EN;
    // Set how often chip will perform a set operation
    flags |= MMC5983_PRD_SET_0 | MMC5983_PRD_SET_1 | MMC5983_PRD_SET_2 | MMC5983_EN_PRD_SET;
    success = MagControlRegWrite (pMag, MMC5983_INT_CTRL_2_REG, flags, true);
    GOTO_IF (success == false, error, "Failed to configure MAG INT_CTRL_2 register");

    pMag->isInitialized = true;
    LOG_INFO ("Successfully initialized MAG");
    return eSTATUS_SUCCESS;

error:
    memset (pMag, 0, sizeof (Mag_t));
    memset (s_ControlRegisters, 0, sizeof (s_ControlRegisters));
    return eSTATUS_FAILURE;
}

eSTATUS_t Mag_Update_ (Mag_t* pMag, bool forcePolling, Vec3f* pOutput) {

    bool success = MagUpdateFromPolling (pMag);
    if (!success) {
        LOG_ERROR ("Failed to update MAG data from polling");
        return eSTATUS_FAILURE;
    }
    *pOutput = MagRaw2NormedGauss (pMag, pMag->rawData, true);
    return eSTATUS_SUCCESS;
}