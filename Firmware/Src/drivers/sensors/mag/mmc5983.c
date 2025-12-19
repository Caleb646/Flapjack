#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "core/core.h"

#include "drivers/driver.h"

#include "drivers/bus/bus.h"
#include "drivers/bus/bus_defs.h"

#include "drivers/bus/spi.h"
#include "drivers/bus/spi_defs.h"

#include "drivers/sensors/inertial/inertial.h"
#include "drivers/sensors/inertial/mmc5983.h"

#include "cfg/sensors/sensor.h"

#define CONTROL_REG2IDX(REG) ((REG) - MMC5983_INT_CTRL_0_REG)

static eSTATUS_t g_ControlRegisters[4] TARG_SHARED_MEM_SECTION = { 0 };

FJ_STATIC eSTATUS_t Mmc5983_WriteControlReg (BusDeviceSPI_t* pBusDeviceSpi, uint8_t reg, uint8_t bitMask, bool doWrite) {

    uint8_t* pValue = &g_ControlRegisters[CONTROL_REG2IDX (reg)];
    *pValue |= bitMask;
    if (doWrite) {
        eSTATUS_t status = SPI_WriteRegister (pBusDeviceSpi, reg, pValue, 1U);
        if (FJ_FAIL (status)) {
            *pValue &= ~(uint32_t)bitMask; // unset the bit on failure
            LOG_ERROR ("Failed to write MAG control register");
            return status;
        }
    }
    return eSTATUS_OK;
}


FJ_STATIC eSTATUS_t Mmc5983_SoftReset (BusDeviceSPI_t* pBusDeviceSpi) {

    // eMAG_STATUS_t status = eSTATUS_OK;
    memset (g_ControlRegisters, 0, sizeof (g_ControlRegisters));
    eSTATUS_t status = Mmc5983_WriteControlReg (pBusDeviceSpi, MMC5983_INT_CTRL_1_REG, MMC5983_SW_RST, true);
    // 10 ms reset time + 5 ms margin
    DelayMicroseconds (15000);
    // Clear the software reset bit on our registers
    status = Mmc5983_WriteControlReg (pBusDeviceSpi, MMC5983_INT_CTRL_1_REG, (uint8_t)~MMC5983_SW_RST, false);
    return status;
}

FJ_STATIC bool Mmc5983_IsMagDataReady (BusDeviceSPI_t* pBusDeviceSpi) {

    uint8_t statusData = 0;
    eSTATUS_t eStatus  = SPI_ReadRegister (pBusDeviceSpi, MMC5983_STATUS_REG, &statusData, 1U);
    if (FJ_OK (eStatus)) {
        return (statusData & MMC5983_MEAS_M_DONE) > 0 ? true : false;
    }
    return false;
}

FJ_STATIC eSTATUS_t Mmc5983_ReadMagData (MagDevice_t* pMagDevice, bool forcePolling, int16_t* pOutData) {

    if (!pMagDevice || !pMagDevice->pBusDevice || !pOutData) {
        return eSTATUS_NULL_ARG;
    }

    uint16_t timeout = 10000U; // 50 ms timeout
    while (forcePolling && !Mmc5983_IsMagDataReady (&(pMagDevice->pBusDevice->spi)) && timeout-- > 0) {
        DelayMicroseconds (5);
    }

    uint8_t pXYZ[7] = { 0 };
    eSTATUS_t status = SPI_ReadRegister (&(pMagDevice->pBusDevice->spi), MMC5983_X_OUT_0_REG, pXYZ, 7U);
    RETURN_IF (FJ_FAIL (status), status, "Failed to read MAG magnetometer data");

    uint32_t x1 = 0, x2 = 0, y1 = 0, y2 = 0, z1 = 0, z2 = 0, xyz = 0;
    x1 = pXYZ[0], x2 = pXYZ[1];
    y1 = pXYZ[2], y2 = pXYZ[3];
    z1 = pXYZ[4], z2 = pXYZ[5];
    xyz         = pXYZ[6];
    pOutData[0] = (((x1 << 8U) | x2) << 2U) | (xyz >> 6U);
    pOutData[1] = (((y1 << 8U) | y2) << 2U) | ((xyz >> 4U) & 0x3U);
    pOutData[2] = (((z1 << 8U) | z2) << 2U) | ((xyz >> 2U) & 0x3U);

    return eSTATUS_OK;
}

FJ_STATIC eSTATUS_t Mmc5983_Init (BusDeviceSPI_t* pBusDeviceSpi) {

    if (!pBusDeviceSpi) {
        return eSTATUS_NULL_ARG;
    }

    eSTATUS_t status = Mmc5983_SoftReset (pBusDeviceSpi);
    GOTO_IF (FJ_FAIL (status), error, "Failed to soft reset MAG");

    uint8_t chipId = 0;
    status         = SPI_ReadRegister (pBusDeviceSpi, MMC5983_PROD_ID_REG, &chipId, 1U);
    GOTO_IF (FJ_FAIL (status), error, "Failed to read MAG chip ID");
    GOTO_IF (chipId != MMC5983_PROD_ID, error, "Invalid MAG chip ID");

    bool success  = true;
    uint8_t flags = 0;
    flags |= MMC5983_AUTO_SR_EN; // Enable automatic set/reset
    // flags |= MMC5983_INT_MEAS_DONE_EN; // Enable measurement done interrupt
    status = Mmc5983_WriteControlReg (pBusDeviceSpi, MMC5983_INT_CTRL_0_REG, flags, true);
    GOTO_IF (FJ_FAIL (status), error, "Failed to configure MAG INT_CTRL_0 register");

    flags  = MMC5983_BW0 | MMC5983_BW1; // Set bandwidth to 800Hz
    status = Mmc5983_WriteControlReg (pBusDeviceSpi, MMC5983_INT_CTRL_1_REG, flags, true);
    GOTO_IF (FJ_FAIL (status), error, "Failed to configure MAG INT_CTRL_1 register");
    // Set continuous measurement mode with 1000Hz output rate
    flags = MMC5983_CM_FREQ_0 | MMC5983_CM_FREQ_1 | MMC5983_CM_FREQ_2 | MMC5983_CMM_EN;
    // Set how often chip will perform a set operation
    flags |= MMC5983_PRD_SET_0 | MMC5983_PRD_SET_1 | MMC5983_PRD_SET_2 | MMC5983_EN_PRD_SET;
    status = Mmc5983_WriteControlReg (pBusDeviceSpi, MMC5983_INT_CTRL_2_REG, flags, true);
    GOTO_IF (FJ_FAIL (status), error, "Failed to configure MAG INT_CTRL_2 register");

    // TODO: Configure EXTI
    // if (pExti != NULL) {

    //     pMag->usingEXTIInterrupt = true;
    //     flags |= MMC5983_AUTO_SR_EN;       // Enable automatic set/reset
    //     flags |= MMC5983_INT_MEAS_DONE_EN; // Enable measurement done interrupt
    //     success = MagControlRegWrite (pMag, MMC5983_INT_CTRL_0_REG, flags, true);
    //     // Configure EXTI
    //     // status = ExtiInit (*pExti);
    //     // GOTO_IF (FJ_FAIL (status), error, "Failed to initialize MAG EXTI");
    //     // status = ExtiAttachDeviceInterrupt (pExti->extiLine, (void*)pMag, MagUpdateFromINT);
    //     // GOTO_IF (FJ_FAIL (status), error, "Failed to attach MAG EXTI interrupt");
    // }

    LOG_INFO ("Successfully initialized MAG");
    return eSTATUS_OK;

error:
    return status;
}

eSTATUS_t Mmc5983_InitMag (MagCfg_t* pMagCfg, MagDevice_t* pOutMagDevice) {

    if (!pMagCfg || !pOutMagDevice || !pOutMagDevice->pBusDevice) {
        return eSTATUS_NULL_ARG;
    }

    if (pMagCfg->type != INER_INTERFACE_ID_MMC5983) {
        LOG_ERROR ("Invalid magnetometer type %d", pMagCfg->type);
        return eSTATUS_INVALID_ARG;
    }

    if (pMagCfg->busCfg.busType != eBUS_TYPE_SPI) {
        LOG_ERROR ("Invalid bus type %d, only SPI is supported", pMagCfg->busCfg.busType);
        return eSTATUS_INVALID_ARG;
    }

    eSTATUS_t status = Mmc5983_Init (&pOutMagDevice->pBusDevice->spi);
    RETURN_IF (FJ_FAIL (status), status, "Failed to initialize MMC5983 magnetometer");
    // NOTE: scale factor is hardcoded for +/- 8 Gauss range and 14-bit resolution
    pOutMagDevice->scaleFactor  = 0.488F; // in microTesla per LSB
    pOutMagDevice->sampleRateHz = 1000;

    pOutMagDevice->vtbl.fnMagReadData = Mmc5983_ReadMagData;

    return status;
}