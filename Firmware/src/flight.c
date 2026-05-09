#include "flight.h"

#include "core/core.h"

#include "mc/filter.h"

#include "device/imu/imu.h"

#include "drivers/sensors/mag/mag.h"

FJ_DEFINE_SHARED (Flight_t, g_Flight) = {
    .max = { [AXIS_IDX_ROLL] = 45.0F, [AXIS_IDX_PITCH] = 45.0F, [AXIS_IDX_YAW] = 180.0F, [AXIS_IDX_THROTTLE] = 1.0F },
    .attitudeFilter = {
        .cfg = {
            .gyroMeasureErrorDegs = CFG_GYRO_MEASURE_ERROR_DEGS,
            .gyroMeasureDriftDegs = CFG_GYRO_MEASURE_DRIFT_DEGS,
        },
    },
};

eSTATUS_t Fc_WarmUp_ (Flight_t* pFlight, uint32_t msWarmUpTime, IMU_t* pIMU, Mag_t* pMag) {

    if (!pFlight) {
        return eSTATUS_FAILURE;
    }

    if (!pIMU && !pMag) {
        return eSTATUS_FAILURE;
    }

    Vec3f attitude            = { 0.0F };
    uint32_t warmUpIterations = 250;
    float msStartTime         = (float)GetMilliseconds ();
    for (uint32_t i = 0; i < warmUpIterations; ++i) {

        Vec3f accel      = { 0.0F };
        Vec3f gyro       = { 0.0F };
        Vec3f mag        = { 0.0F };
        eSTATUS_t status = IMU_Update (pIMU, true, &accel, &gyro);
        RETURN_IF (status != eSTATUS_SUCCESS, status, "Failed to poll IMU");

        if (pMag) {
            status = Mag_Update_ (pMag, true, &mag);
            RETURN_IF (status != eSTATUS_SUCCESS, status, "Failed to poll Mag");
        }

        float dt    = ((float)GetMilliseconds () - msStartTime) / 1000.0F;
        msStartTime = (float)GetMilliseconds ();
        if (!pMag) {
            MadgwickFilter_Update (&pFlight->attitudeFilter, &accel, &gyro, NULL, dt, &attitude);
        } else {
            MadgwickFilter_Update (&pFlight->attitudeFilter, &accel, &gyro, &mag, dt, &attitude);
        }
    }

    pFlight->current[AXIS_IDX_ROLL]  = attitude.roll;
    pFlight->current[AXIS_IDX_PITCH] = attitude.pitch;
    pFlight->current[AXIS_IDX_YAW]   = attitude.yaw;
    return eSTATUS_SUCCESS;
}

eSTATUS_t Fc_Init_ (Flight_t* pOutFlight) {

    eSTATUS_t status = MadgwickFilter_Init (&pOutFlight->attitudeFilter);
    RETURN_IF (status != eSTATUS_SUCCESS, status, "Failed to initialize attitude filter");

    return Fc_WarmUp_ (pOutFlight, 250U, Imu_Get (), Mag_Get ());
}

void Fc_LogData_ (Flight_t* pFlightData) {

    LOG_4_FLOATS (
    LOG_DATA_TYPE_ATTITUDE,
    roll,
    pFlightData->current[AXIS_IDX_ROLL],
    pitch,
    pFlightData->current[AXIS_IDX_PITCH],
    yaw,
    pFlightData->current[AXIS_IDX_YAW],
    throttle,
    pFlightData->current[AXIS_IDX_THROTTLE]
    );
}