#ifndef AERO_FLIGHT_H
#define AERO_FLIGHT_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "cfg/cfg.h"

typedef struct Madgwick_s {
    // estimated orientation quaternion elements with initial conditions
    Vec4f qEst;
    // gyro bias error
    Vec3f gbias;
    // Gamma_t (γt)
    float beta;
    float zeta;
    // reference direction of flux in earth frame
    float bx;
    float bz;
    uint32_t usLastUpdate;
} Madgwick_t;

typedef struct FlightCfg_s {
    float gyroMeasureErrorDegs;
    float gyroMeasureDriftDegs;
    // reference direction of flux in earth frame
    float bx;
    float bz;
} FlightCfg_t;

typedef struct FlightData_s {
    float currentAttitude[AXIS_IDX_COUNT];
    float targetAttitude[AXIS_IDX_COUNT];

    float currentAltitude;
    float targetAltitude;

    float currentThrottle;
    float targetThrottle;

    union {
        Madgwick_t madgwick;
    };
} FlightData_t;

CFG_DECLARE (FlightCfg_t, FlightCfg);
FJ_DECLARE_SHARED (FlightData_t, e_FlightData);

static inline FlightData_t const* FlightData_Get (void) {
    return &e_FlightData;
}
eSTATUS_t FlightData_Init (bool doWarmUp);

eSTATUS_t Attitude_Update (uint32_t usCurrentTime);

eSTATUS_t Altitude_Update (uint32_t usCurrentTime);

#endif // AERO_FLIGHT_H