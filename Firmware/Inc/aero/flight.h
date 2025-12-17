#ifndef AERO_FLIGHT_H
#define AERO_FLIGHT_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "cfg/cfg.h"

typedef struct Madgwick_s Madgwick_t;

typedef struct FlightCfg_s {
    float gyroMeasureErrorDegs;
    float gyroMeasureDriftDegs;
    // reference direction of flux in earth frame
    float bx;
    float bz;
} FlightCfg_t;

typedef struct Flight_s {
    Vec3f attitude;
    float altitude;
    Madgwick_t* pMadgwick;
} Flight_t;

CFG_DECLARE (FlightCfg_t, FlightCfg);
FJ_DECLARE_SHARED (Flight_t, e_Flight);

static inline Vec3f* Attitude_GetMutable (void) {
    return &e_Flight.attitude;
}
eSTATUS_t Attitude_Init (bool doWarmUp);
eSTATUS_t Attitude_Update (float dt);

eSTATUS_t Altitude_Update (float dt);

#endif // AERO_FLIGHT_H