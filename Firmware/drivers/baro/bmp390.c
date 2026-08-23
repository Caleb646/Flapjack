/*
 * BMP390 barometer backend. Sits on SPI5, sharing the bus with the magnetometer
 * on its own NSS (target/flapjack_v1/flapjack_v1.h).
 *
 * Configured from the "Drone" row of datasheet Table 10 (recommended settings
 * per use case): normal mode, pressure oversampling x8, temperature x1, IIR
 * filter 2, ODR 50 Hz. That table's filter column is a register value, not a
 * coefficient - the coefficient set is {0,1,3,7,15,...}, so 2 means coef_3.
 * The x8/x1 pair takes ~18.9 ms to convert against the 20 ms ODR period, which
 * is the margin Bosch's own driver requires; raising oversampling without also
 * slowing the ODR would make the part flag conf_err and silently drop samples.
 *
 * The part free-runs in normal mode and Read never waits on it: it checks drdy
 * once and reports eSTATUS_BUSY when the newest sample is one it has already
 * returned. Sampling rate is Baro_Task's to choose, not the driver's.
 */

#include "hal.h"
#include "target.h"

#include "core/core.h"

#include "drivers/bus/spi.h"

#include "drivers/baro/barodrv.h"

#include "drivers/io/exti.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#if defined(BARO_SPI_BUS_ID)

#define BMP390_CHIP_ID        0x60U
#define BMP390_CMD_SOFT_RESET 0xB6U

#define BMP390_REG_CHIP_ID    0x00U
#define BMP390_REG_STATUS     0x03U
#define BMP390_REG_DATA_0     0x04U
#define BMP390_REG_INT_CTRL   0x19U
#define BMP390_REG_PWR_CTRL   0x1BU
#define BMP390_REG_OSR        0x1CU
#define BMP390_REG_ODR        0x1DU
#define BMP390_REG_CONFIG     0x1FU
#define BMP390_REG_NVM_PAR_T1 0x31U
#define BMP390_REG_CMD        0x7EU

#define BMP390_DRDY_PRESS     (1U << 5U)

/* INT_CTRL. int_latch (bit 2) deliberately left clear: a pulsed interrupt
 * self-clears, so nothing has to read INT_STATUS to re-arm the line. int_od
 * (bit 0) clear selects push-pull. */
#define BMP390_INT_LEVEL_HIGH (1U << 1U)
#define BMP390_INT_DRDY_EN    (1U << 6U)

#define BMP390_PRESS_EN       (1U << 0U)
#define BMP390_TEMP_EN        (1U << 1U)
#define BMP390_MODE_NORMAL    (3U << 4U)

// osr_p occupies bits 2..0 and osr_t bits 5..3; each field encodes 2^value.
#define BMP390_OSR_P_X8       (3U << 0U)
#define BMP390_OSR_T_X1       (0U << 3U)

// odr_sel subdivides the 200 Hz base rate by 2^odr_sel.
#define BMP390_ODR_50_HZ      0x02U

// iir_filter occupies bits 3..1.
#define BMP390_IIR_COEF_3     (2U << 1U)

#define CALIB_LEN             21U // NVM_PAR_T1 (0x31) .. NVM_PAR_P11 (0x45)
#define DATA_LEN              6U  // pressure then temperature, 24-bit LSB first
#define SPI_DUMMY_BYTES       1U
#define RX_BUFFER_SZ          (CALIB_LEN + SPI_DUMMY_BYTES)

#define RESET_SETTLE_US       2000U // datasheet gives no figure; Bosch's driver waits 2 ms

typedef struct Bmp390Calib_s {
    float t1, t2, t3;
    float p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
    float tLin; // compensated temperature, carried into the pressure polynomial
} Bmp390Calib_t;

typedef struct Bmp390_s {
    SpiDev_t spiDev;
    Bmp390Calib_t calib;
} Bmp390_t;

/*
 * A BMP390 SPI read returns a dummy byte before the register contents, so the
 * transfer is one byte longer than the caller asked for. SpiDev_ReadRegister
 * has no notion of that, hence the local buffer - same shape as IMUReadReg in
 * drivers/imu/bmi323.c.
 */
STATIC eSTATUS_t BaroRead (Bmp390_t* pBaro, uint8_t reg, uint8_t* pData, uint16_t size) {

    uint8_t pRx[RX_BUFFER_SZ] = { 0 };
    uint16_t totalSize        = size + SPI_DUMMY_BYTES;
    if (totalSize > RX_BUFFER_SZ) {
        return eSTATUS_INVALID_ARG;
    }

    eSTATUS_t status = SpiDev_ReadRegister (&pBaro->spiDev, reg, pRx, totalSize);
    DelayMicroseconds (2);
    if (status != eSTATUS_SUCCESS) {
        return status;
    }
    memcpy (pData, &pRx[SPI_DUMMY_BYTES], size);
    return eSTATUS_SUCCESS;
}

STATIC eSTATUS_t BaroWrite (Bmp390_t* pBaro, uint8_t reg, uint8_t value) {

    eSTATUS_t status = SpiDev_WriteRegister (&pBaro->spiDev, reg, &value, 1U);
    DelayMicroseconds (2);
    return status;
}

STATIC eSTATUS_t BaroReadCalibration (Bmp390_t* pBaro) {

    uint8_t raw[CALIB_LEN] = { 0 };
    eSTATUS_t status       = BaroRead (pBaro, BMP390_REG_NVM_PAR_T1, raw, CALIB_LEN);
    if (status != eSTATUS_SUCCESS) {
        return status;
    }

    // Widths and signs are Table 24; every coefficient is little-endian.
    uint16_t nvmT1 = (uint16_t)(((uint16_t)raw[1] << 8U) | raw[0]);
    uint16_t nvmT2 = (uint16_t)(((uint16_t)raw[3] << 8U) | raw[2]);
    int8_t nvmT3   = (int8_t)raw[4];
    int16_t nvmP1  = (int16_t)(((uint16_t)raw[6] << 8U) | raw[5]);
    int16_t nvmP2  = (int16_t)(((uint16_t)raw[8] << 8U) | raw[7]);
    int8_t nvmP3   = (int8_t)raw[9];
    int8_t nvmP4   = (int8_t)raw[10];
    uint16_t nvmP5 = (uint16_t)(((uint16_t)raw[12] << 8U) | raw[11]);
    uint16_t nvmP6 = (uint16_t)(((uint16_t)raw[14] << 8U) | raw[13]);
    int8_t nvmP7   = (int8_t)raw[15];
    int8_t nvmP8   = (int8_t)raw[16];
    int16_t nvmP9  = (int16_t)(((uint16_t)raw[18] << 8U) | raw[17]);
    int8_t nvmP10  = (int8_t)raw[19];
    int8_t nvmP11  = (int8_t)raw[20];

    /*
     * Datasheet 8.4 scales each NVM word into its float coefficient by a power
     * of two. They are applied here as exact hex-float multipliers (0x1pN is
     * 2^N) rather than decimal divisors, because the largest of them - 2^65 -
     * has no decimal literal a reader can check by eye.
     */
    Bmp390Calib_t* pCal = &pBaro->calib;
    pCal->t1            = (float)nvmT1 * 0x1p8f;
    pCal->t2            = (float)nvmT2 * 0x1p-30f;
    pCal->t3            = (float)nvmT3 * 0x1p-48f;
    pCal->p1            = ((float)nvmP1 - 0x1p14f) * 0x1p-20f;
    pCal->p2            = ((float)nvmP2 - 0x1p14f) * 0x1p-29f;
    pCal->p3            = (float)nvmP3 * 0x1p-32f;
    pCal->p4            = (float)nvmP4 * 0x1p-37f;
    pCal->p5            = (float)nvmP5 * 0x1p3f;
    pCal->p6            = (float)nvmP6 * 0x1p-6f;
    pCal->p7            = (float)nvmP7 * 0x1p-8f;
    pCal->p8            = (float)nvmP8 * 0x1p-15f;
    pCal->p9            = (float)nvmP9 * 0x1p-48f;
    pCal->p10           = (float)nvmP10 * 0x1p-48f;
    pCal->p11           = (float)nvmP11 * 0x1p-65f;
    return eSTATUS_SUCCESS;
}

// Datasheet 8.5. Also latches tLin, which the pressure polynomial needs.
STATIC float BaroCompensateTemperature (Bmp390Calib_t* pCal, uint32_t rawTemp) {

    float partial1 = (float)rawTemp - pCal->t1;
    float partial2 = partial1 * pCal->t2;
    pCal->tLin     = partial2 + (partial1 * partial1) * pCal->t3;
    return pCal->tLin;
}

// Datasheet 8.6.
STATIC float BaroCompensatePressure (Bmp390Calib_t const* pCal, uint32_t rawPress) {

    float tLin  = pCal->tLin;
    float press = (float)rawPress;

    float partial1 = pCal->p6 * tLin;
    float partial2 = pCal->p7 * (tLin * tLin);
    float partial3 = pCal->p8 * (tLin * tLin * tLin);
    float out1     = pCal->p5 + partial1 + partial2 + partial3;

    partial1   = pCal->p2 * tLin;
    partial2   = pCal->p3 * (tLin * tLin);
    partial3   = pCal->p4 * (tLin * tLin * tLin);
    float out2 = press * (pCal->p1 + partial1 + partial2 + partial3);

    partial1       = press * press;
    partial2       = pCal->p9 + pCal->p10 * tLin;
    partial3       = partial1 * partial2;
    float partial4 = partial3 + (press * press * press) * pCal->p11;

    return out1 + out2 + partial4;
}

STATIC bool Bmp390_IsDataReady (void* ctx) {

    Bmp390_t* pBaro = (Bmp390_t*)ctx;
    if (!pBaro) {
        return false;
    }

    uint8_t status = 0;
    if (BaroRead (pBaro, BMP390_REG_STATUS, &status, 1U) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to read BARO status register");
        return false;
    }
    return (status & BMP390_DRDY_PRESS) > 0U;
}

STATIC eSTATUS_t Bmp390_HwInit (Bmp390_t* pBaro, DataReadySignal_t const* pSignal) {

    bool useInterrupt = (pSignal != NULL) && (pSignal->Notify != NULL);

    pBaro->spiDev.cfg.busId    = BARO_SPI_BUS_ID;
    pBaro->spiDev.cfg.pNssPort = BARO_SPI_NSS_GPIO_PORT;
    pBaro->spiDev.cfg.nssPin   = BARO_SPI_NSS_GPIO_PIN;
    if (SpiDev_Init (&pBaro->spiDev) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize SPI device for BARO");
        return eSTATUS_FAILURE;
    }

    GOTO_IF (BaroWrite (pBaro, BMP390_REG_CMD, BMP390_CMD_SOFT_RESET) != eSTATUS_SUCCESS, error, "Failed to soft reset BARO");
    DelayMicroseconds (RESET_SETTLE_US);

    uint8_t chipId = 0;
    GOTO_IF (BaroRead (pBaro, BMP390_REG_CHIP_ID, &chipId, 1U) != eSTATUS_SUCCESS, error, "Failed to read BARO chip ID");
    GOTO_IF (chipId != BMP390_CHIP_ID, error, "Invalid BARO chip ID");

    // The coefficients survive the soft reset, but reading them after it keeps
    // the NVM access away from a part that may still be powering up.
    GOTO_IF (BaroReadCalibration (pBaro) != eSTATUS_SUCCESS, error, "Failed to read BARO calibration coefficients");

    GOTO_IF (BaroWrite (pBaro, BMP390_REG_OSR, BMP390_OSR_P_X8 | BMP390_OSR_T_X1) != eSTATUS_SUCCESS, error, "Failed to configure BARO oversampling");
    GOTO_IF (BaroWrite (pBaro, BMP390_REG_ODR, BMP390_ODR_50_HZ) != eSTATUS_SUCCESS, error, "Failed to configure BARO output data rate");
    GOTO_IF (BaroWrite (pBaro, BMP390_REG_CONFIG, BMP390_IIR_COEF_3) != eSTATUS_SUCCESS, error, "Failed to configure BARO IIR filter");

    if (useInterrupt) {
        uint8_t intCtrl = BMP390_INT_DRDY_EN | BMP390_INT_LEVEL_HIGH;
        GOTO_IF (BaroWrite (pBaro, BMP390_REG_INT_CTRL, intCtrl) != eSTATUS_SUCCESS, error, "Failed to configure BARO interrupt output");
    }

    // Last: leaving sleep starts conversions, so the rate settings above have to
    // already be in place or the part runs at its reset defaults.
    uint8_t pwrCtrl = BMP390_PRESS_EN | BMP390_TEMP_EN | BMP390_MODE_NORMAL;
    GOTO_IF (BaroWrite (pBaro, BMP390_REG_PWR_CTRL, pwrCtrl) != eSTATUS_SUCCESS, error, "Failed to enable BARO measurement");

#if BRD_IS_ENABLED (BARO_INT)
    if (useInterrupt) {
        ExtiConf_t extiConf = {
            .pPort       = BRD_GET_GPIO_PORT (BARO, INT),
            .pin         = BRD_GET_GPIO_PIN (BARO, INT),
            .trigger     = GPIO_MODE_IT_RISING,   // int_level high, pulsed - see BMP390_INT_LEVEL_HIGH
            .pull        = GPIO_NOPULL,
            .irqPriority = pSignal->irqPriority,
            .callback    = pSignal->Notify,
            .ctx         = pSignal->ctx,
        };
        GOTO_IF (Exti_Register (&extiConf) != eSTATUS_SUCCESS, error, "Failed to claim BARO data ready EXTI line");
    }
#else
    if (useInterrupt) {
        LOG_WARN ("Board declares no BARO_INT pin; baro data ready stays polled");
    }
#endif

    LOG_INFO ("Successfully initialized BARO");
    return eSTATUS_SUCCESS;

error:
    memset (pBaro, 0, sizeof (Bmp390_t));
    return eSTATUS_FAILURE;
}

STATIC eSTATUS_t Bmp390_Read (void* ctx, bool forcePolling, BaroData_t* pOutData) {

    FJ_UNUSED (forcePolling);
    Bmp390_t* pBaro = (Bmp390_t*)ctx;
    if (!pBaro || !pOutData) {
        return eSTATUS_NULL_ARG;
    }

    // One drdy check, no wait. Reporting BUSY rather than re-reading stops a
    // sample being published twice: Nav_Update applies its baro correction once
    // per arriving sample, so a duplicate would double that sample's gain.
    if (Bmp390_IsDataReady (pBaro) == false) {
        return eSTATUS_BUSY;
    }

    // One burst: the part only shadows the data registers for the length of a
    // single read, so splitting this up can mix two measurements (3.10.1).
    uint8_t raw[DATA_LEN] = { 0 };
    eSTATUS_t status      = BaroRead (pBaro, BMP390_REG_DATA_0, raw, DATA_LEN);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to read BARO data registers");
        return status;
    }

    uint32_t rawPress = ((uint32_t)raw[2] << 16U) | ((uint32_t)raw[1] << 8U) | raw[0];
    uint32_t rawTemp  = ((uint32_t)raw[5] << 16U) | ((uint32_t)raw[4] << 8U) | raw[3];

    // Temperature first - it sets the tLin that pressure compensation reads.
    pOutData->temperatureC = BaroCompensateTemperature (&pBaro->calib, rawTemp);
    pOutData->pressurePa   = BaroCompensatePressure (&pBaro->calib, rawPress);
    return eSTATUS_SUCCESS;
}

eSTATUS_t BaroDrv_Init (BaroDriver_t* pOutDriver) {

    if (!pOutDriver) {
        return eSTATUS_NULL_ARG;
    }

    /* No memset of pOutDriver: cfg is the caller's input and lives in the same
     * struct, so clearing it here would erase what this function is reading. */
    pOutDriver->ctx = Allocate (sizeof (Bmp390_t));
    if (!pOutDriver->ctx) {
        return eSTATUS_FAILURE;
    }
    pOutDriver->Read        = Bmp390_Read;
    pOutDriver->IsDataReady = Bmp390_IsDataReady;

    Bmp390_t* pBaro = (Bmp390_t*)pOutDriver->ctx;
    memset (pBaro, 0, sizeof (Bmp390_t));

    return Bmp390_HwInit (pBaro, &pOutDriver->cfg.signal);
}

#else // board declares no barometer wiring

eSTATUS_t BaroDrv_Init (BaroDriver_t* pOutDriver) {

    FJ_UNUSED (pOutDriver);
    return eSTATUS_UNSUPPORTED;
}

#endif // BARO_SPI_BUS_ID
