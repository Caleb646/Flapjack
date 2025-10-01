#include "device/mag/mag.h"
#include "common.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "conf/ids.h"
#include "device/mag/mmc5983.h"
#include "hal.h"
#include "log/logger.h"
#include "mem/mem.h"
#include "peripheral/bus/bus.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define MAX_BUFFER_SIZE      12U
#define NCONTROL_REGISTERS   4U
#define CONTROL_REG2IDX(REG) ((REG) - MMC5983_INT_CTRL_0_REG)
#define MAG_UNSIGNED_MAX     ((1U << 18U) - 1U)
#define MAG_SIGNED_POS_MAX   (1U << 17U)
#define MAG_VALID(pMAG)      ((pMAG) != NULL && (pMAG)->isInitialized == true)

static SHARED_MEM_SECTION Mag_t gMag = { 0 };
static SHARED_MEM_SECTION uint8_t gControlRegisters[NCONTROL_REGISTERS] = { 0 };

STATIC_TESTABLE_DECL eMAG_STATUS_t MagRead (vMag_t* pMag, uint8_t reg, uint8_t* pData, uint16_t size) {

    eBUS_ID_t busId       = pMag->busId;
    eDEVICE_ID_t deviceId = pMag->deviceId;

    size_t totalSize = size + pMag->nBusDummyBytes + 1U;
    if (totalSize > MAX_BUFFER_SIZE) {
        LOG_ERROR ("Read size exceeds max buffer size");
        return eMAG_BUS_ERROR;
    }

    uint8_t txData[MAX_BUFFER_SIZE] = { reg | 0x80U }; // Set MSB for read operation
    uint8_t rxData[MAX_BUFFER_SIZE] = { 0 };

    eSTATUS_t status = BUS_WRITE_READ (pMag->bus, txData, rxData, totalSize);
    if (status != eSTATUS_SUCCESS) {
        return eMAG_BUS_ERROR;
    }

    memcpy (pData, &(rxData[pMag->nBusDummyBytes + 1U]), size);
    DelayMicroseconds (2);
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eMAG_STATUS_t MagWrite (vMag_t* pMag, uint8_t reg, uint8_t* pData, uint16_t size) {

    eBUS_ID_t busId       = pMag->busId;
    eDEVICE_ID_t deviceId = pMag->deviceId;
    uint16_t totalSize    = size + 1U;

    if (totalSize > MAX_BUFFER_SIZE) {
        LOG_ERROR ("Write size exceeds max buffer size");
        return eMAG_BUS_ERROR;
    }

    uint8_t txData[MAX_BUFFER_SIZE] = { 0 };
    txData[0] = reg & 0x7FU; // Clear MSB for write operation
    memcpy (&(txData[1]), pData, size);

    eSTATUS_t status = BUS_WRITE (pMag->bus, txData, totalSize);
    if (status != eSTATUS_SUCCESS) {
        return eMAG_BUS_ERROR;
    }

    DelayMicroseconds (2);
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL bool
MagControlRegWrite (vMag_t* pMag, uint8_t reg, uint8_t bitMask, bool doWrite) {

    uint8_t* pValue = &gControlRegisters[CONTROL_REG2IDX (reg)];
    *pValue |= bitMask;
    if (doWrite == true) {
        if (MagWrite (pMag, reg, pValue, 1U) != eSTATUS_SUCCESS) {
            *pValue &= ~bitMask; // unset the bit on failure
            LOG_ERROR ("Failed to write MAG control register");
            return false;
        }
    }
    return true;
}

STATIC_TESTABLE_DECL uint8_t MagReadStatusReg (vMag_t* pMag) {

    uint8_t status = 0;
    if (MagRead (pMag, MMC5983_STATUS_REG, &status, 1U) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to read MAG status register");
        return 0;
    }
    return status;
}

STATIC_TESTABLE_DECL bool MagXYZIsReady (vMag_t* pMag) {

    return (MagReadStatusReg (pMag) & MMC5983_MEAS_M_DONE) > 0 ? true : false;
}

STATIC_TESTABLE_DECL eMAG_STATUS_t MagSoftReset (vMag_t* pMag) {

    // eMAG_STATUS_t status = eSTATUS_SUCCESS;
    memset (gControlRegisters, 0, sizeof (gControlRegisters));

    bool success =
    MagControlRegWrite (pMag, MMC5983_INT_CTRL_1_REG, MMC5983_SW_RST, true);
    // 10 ms reset time + 5 ms margin
    DelayMicroseconds (15000);
    // Clear the software reset bit on our registers
    MagControlRegWrite (pMag, MMC5983_INT_CTRL_1_REG, (uint8_t)~MMC5983_SW_RST, false);
    return success == true ? eSTATUS_SUCCESS : eMAG_BUS_ERROR;
}

/*
 * Could be called inside an interrupt
 */
STATIC_TESTABLE_DECL bool MagUpdateRawData (vMag_t* pMag) {

    uint8_t pXYZ[7]  = { 0 };
    eSTATUS_t status = MagRead (pMag, MMC5983_X_OUT_0_REG, pXYZ, 7U);
    if (status != eSTATUS_SUCCESS) {
        return false;
    }

    uint32_t x1, x2, y1, y2, z1, z2, xyz;
    x1 = pXYZ[0], x2 = pXYZ[1];
    y1 = pXYZ[2], y2 = pXYZ[3];
    z1 = pXYZ[4], z2 = pXYZ[5];
    xyz               = pXYZ[6];
    pMag->rawData.x   = (((x1 << 8U) | x2) << 2U) | (xyz >> 6U);
    pMag->rawData.y   = (((y1 << 8U) | y2) << 2U) | ((xyz >> 4U) & 0x3U);
    pMag->rawData.z   = (((z1 << 8U) | z2) << 2U) | ((xyz >> 2U) & 0x3U);
    pMag->dataUpdated = true;
    pMag->msLastUpdateTime = GetMilliseconds ();
    return true;
}

STATIC_TESTABLE_DECL Vec3f MagRaw2NormedGauss (vMag_t const* pMag, Vec3u raw, bool doNormalization) {

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
STATIC_TESTABLE_DECL UNUSED_FN_DECL bool MagUpdateFromINT (vMag_t* pMag) {

    if (MagXYZIsReady (pMag) == false) {
        return false;
    }
    if (MagUpdateRawData (pMag) == false) {
        return false;
    }
    return true;
}

STATIC_TESTABLE_DECL bool MagUpdateFromPolling (vMag_t* pMag) {

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

eMAG_STATUS_t MagInit (MagInitConf_t conf, Mag_t* pOutMag, BusInterface_t* pBusOverride) {

    eSTATUS_t status         = eSTATUS_SUCCESS;
    DeviceBoardConf_t device = conf.boardConf;
    eDEVICE_ID_t deviceId    = device.deviceId;

    BusBoardConf_t* pBus   = device.generic.pBusBoardConf;
    EXTIBoardConf_t* pExti = device.generic.pExtiBoardConf;
    RETURN_IF_NULL (pBus, eMAG_INVALID_DEVICE, "MAG bus configuration is NULL");
    eBUS_ID_t busId = pBus->busId;

    vMag_t* pMag = &gMag;
    if (pOutMag != NULL) {
        pMag = pOutMag;
    }
    memset (pMag, 0, sizeof (vMag_t));
    pMag->deviceId = deviceId;
    pMag->busId    = busId;
    // SPI reads have 1 dummy byte at the beginning
    pMag->nBusDummyBytes = BUS_ID_IS_SPI (busId) == true ? 1U : 0U;

    if (pBusOverride != NULL) {
        pMag->bus = *pBusOverride;
    } else {
        BUS_INIT (&status, device, *pBus, &pMag->bus);
        GOTO_IF (STATUS_FAIL (status), error, "Failed to initialize MAG bus");
    }

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

    flags = MMC5983_BW0 | MMC5983_BW1; // Set bandwidth to 800Hz
    success = MagControlRegWrite (pMag, MMC5983_INT_CTRL_1_REG, flags, true);
    GOTO_IF (success == false, error, "Failed to configure MAG INT_CTRL_1 register");

    // Set continuous measurement mode with 1000Hz output rate
    flags = MMC5983_CM_FREQ_0 | MMC5983_CM_FREQ_1 | MMC5983_CM_FREQ_2 | MMC5983_CMM_EN;
    // Set how often chip will perform a set operation
    flags |= MMC5983_PRD_SET_0 | MMC5983_PRD_SET_1 | MMC5983_PRD_SET_2 | MMC5983_EN_PRD_SET;
    success = MagControlRegWrite (pMag, MMC5983_INT_CTRL_2_REG, flags, true);
    GOTO_IF (success == false, error, "Failed to configure MAG INT_CTRL_2 register");

    // TODO: Configure EXTI
    if (pExti != NULL) {

        pMag->usingEXTIInterrupt = true;
        flags |= MMC5983_AUTO_SR_EN;       // Enable automatic set/reset
        flags |= MMC5983_INT_MEAS_DONE_EN; // Enable measurement done interrupt
        success = MagControlRegWrite (pMag, MMC5983_INT_CTRL_0_REG, flags, true);
        // Configure EXTI
        // status = ExtiInit (*pExti);
        // GOTO_IF (STATUS_FAIL (status), error, "Failed to initialize MAG EXTI");
        // status = ExtiAttachDeviceInterrupt (pExti->extiLine, (void*)pMag, MagUpdateFromINT);
        // GOTO_IF (STATUS_FAIL (status), error, "Failed to attach MAG EXTI interrupt");
    }

    pMag->isInitialized = true;
    LOG_INFO ("Successfully initialized MAG");
    return eSTATUS_SUCCESS;

error:
    memset (pMag, 0, sizeof (vMag_t));
    memset (gControlRegisters, 0, sizeof (gControlRegisters));
    return eSTATUS_FAILURE;
}

eMAG_STATUS_t MagStart (vMag_t* pMag) {

    RETURN_IF (MAG_VALID (pMag) == false, eMAG_INVALID_DEVICE, "MAG device is not initialized");

    eMAG_STATUS_t status = eSTATUS_SUCCESS;
    bool success         = true;
    if (pMag->usingEXTIInterrupt == true) {
        // Enable interrupts on the MAG
        uint8_t flags = MMC5983_INT_MEAS_DONE_EN;
        success = MagControlRegWrite (pMag, MMC5983_INT_CTRL_0_REG, flags, true);
        RETURN_IF (success == false, eMAG_BUS_ERROR, "Failed to configure MAG INT_CTRL_0 register");
    }
    return status;
}

eMAG_STATUS_t MagStop (vMag_t* pMag) {

    RETURN_IF (MAG_VALID (pMag) == false, eMAG_INVALID_DEVICE, "MAG device is not initialized");

    eMAG_STATUS_t status = eSTATUS_SUCCESS;
    bool success         = true;
    if (pMag->usingEXTIInterrupt == true) {
        // Disable interrupts on the MAG
        uint8_t flags = (uint8_t)~MMC5983_INT_MEAS_DONE_EN;
        success = MagControlRegWrite (pMag, MMC5983_INT_CTRL_0_REG, flags, true);
        RETURN_IF (success == false, eMAG_BUS_ERROR, "Failed to configure MAG INT_CTRL_0 register");
    }
    return status;
}

eMAG_STATUS_t MagUpdate (vMag_t* pMag, bool forcePolling, Vec3f* pOutput) {

    RETURN_IF (MAG_VALID (pMag) == false, eMAG_INVALID_DEVICE, "MAG device is not initialized");
    RETURN_IF_NULL (pOutput, eMAG_INVALID_DEVICE, "Output pointer is NULL");

    bool success = true;
    bool usePolling = pMag->usingEXTIInterrupt == false || forcePolling == true;
    // If using interrupts, the interrupt handler should have already updated the data
    if (usePolling == true) {
        success = MagUpdateFromPolling (pMag);
    }

    RETURN_IF (pMag->dataUpdated == false, eMAG_DATA_NOT_READY, "MAG data not ready");
    RETURN_IF (success != true, eMAG_BUS_ERROR, "Failed to update MAG data");
    *pOutput          = MagRaw2NormedGauss (pMag, pMag->rawData, true);
    pMag->dataUpdated = false;
    return eSTATUS_SUCCESS;
}

vMag_t* MagGetActiveDevice (void) {

    if (MAG_VALID (&gMag) == false) {
        LOG_ERROR ("No active valid MAG device");
        return NULL;
    }
    return &gMag;
}