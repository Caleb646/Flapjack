#ifndef FLIGHT_H
#define FLIGHT_H

#include <stdbool.h>
#include <stdint.h>

#include "core/core.h"

#include "mc/filter.h"

#include "device/imu/imu.h"

#include "drivers/sensors/mag/mag.h"

typedef struct Flight_s {

    bool isArmed;

    float current[AXIS_IDX_COUNT];
    float target[AXIS_IDX_COUNT];
    float max[AXIS_IDX_COUNT];

    float currentAltitude;
    float targetAltitude;

    // LowPassFilter_t accelFilter;
    // LowPassFilter_t gyroFilter;
    MadgwickFilter_t attitudeFilter;

} Flight_t;

FJ_DECLARE_SHARED (Flight_t, g_Flight);

eSTATUS_t Fc_WarmUp_ (Flight_t* pFlight, uint32_t msWarmUpTime, IMU_t* pIMU, Mag_t* pMag);

eSTATUS_t Fc_Init_ (Flight_t* pOutFlight);
static inline eSTATUS_t Fc_Init (void) {
    return Fc_Init_ (&g_Flight);
}

void Fc_LogData_ (Flight_t* pFlightData);
static inline void Fc_LogData (void) {
    Fc_LogData_ (&g_Flight);
}

static inline Flight_t* Fc_Get (void) {
    return &g_Flight;
}

static inline bool Fc_IsArmed (void) {
    return g_Flight.isArmed;
}


#endif