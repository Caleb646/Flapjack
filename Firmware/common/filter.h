#ifndef MOTION_CONTROL_FILTER_H
#define MOTION_CONTROL_FILTER_H

#include "core/core.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    struct {
        float alpha;
        bool isFirstUpdate;
    } cfg;
    uint32_t usLastUpdateTime;
} LowPassFilter_t;

/*
 * Vertical estimator: a third-order complementary filter fusing baro-derived
 * altitude with the vertical acceleration recovered from the IMU.
 *
 * Baro alone is quiet in the long run but laggy and metre-noisy sample to
 * sample; integrated accel is smooth and instantaneous but drifts without
 * bound. So predict from accel at the IMU rate, and correct towards the baro at
 * the (much slower) baro rate - which is why predict and correct are separate
 * calls rather than one update: they are driven by different sensors arriving
 * on different clocks, 400 Hz against ~50 Hz.
 *
 * The third state is the accelerometer's vertical bias. Without it a constant
 * offset of even 0.05 m/s^2 - well inside a good part's spec - integrates into
 * a standing velocity error that the altitude correction can only fight, never
 * cancel, leaving a permanent climb-rate bias. Estimating it costs one gain and
 * one float.
 *
 * Sign convention: alt and vz are UP-positive metres and metres/second, because
 * that is what an altitude reads like. NED down-positive is the publish
 * boundary's problem (nav.c negates), not this filter's.
 */
typedef struct {
    struct {
        float kAlt;    // altitude error -> altitude    [1/s]
        float kVel;    // altitude error -> velocity    [1/s^2]
        float kBias;   // altitude error -> accel bias  [1/s^3]
        float maxBias; // clamp on the bias estimate    [m/s^2]
    } cfg;
    float alt;       // metres above the datum, up positive
    float vz;        // metres/second, up positive
    float accelBias; // m/s^2, estimated vertical accelerometer bias
} AltitudeFilter_t;

typedef struct {
    // estimated orientation quaternion elements with initial conditions
    Vec4f qEst;
    // gyro bias error
    Vec3f gbias;
    struct {
        float gyroMeasureErrorDegs;
        float gyroMeasureDriftDegs;
    } cfg;
    // Gamma_t (γt)
    float beta;
    float zeta;
    // reference direction of flux in earth frame
    float bx;
    float bz;
    uint32_t usLastUpdateTime;
} MadgwickFilter_t;

// clang-format off

void MadgwickFilter_Update_6DOF_ (MadgwickFilter_t* pFilter, Vec3f const* pAccel, Vec3f const* pGyroDegs, float dt);
void MadgwickFilter_Update_9DOF_ (MadgwickFilter_t* pFilter, Vec3f const* pAccel, Vec3f const* pGyroDegs, Vec3f const* pMag, float dt);
eSTATUS_t MadgwickFilter_Update (MadgwickFilter_t* pFilter, Vec3f const* pAccel, Vec3f const* pGyro, Vec3f const* pMag, float dt, Vec3f* pOutAttitude);
void MadgwickFilter_QuatToEuler (MadgwickFilter_t const* pFilter, Vec3f* pOutEuler);
eSTATUS_t MadgwickFilter_Init (MadgwickFilter_t* pFilter);

eSTATUS_t LowPassFilter_Init (LowPassFilter_t* pFilter);

float Baro_PressureToAltitude (float pressurePa, float referencePa, float referenceTempC);
eSTATUS_t AltitudeFilter_Init (AltitudeFilter_t* pFilter);
eSTATUS_t AltitudeFilter_Predict (AltitudeFilter_t* pFilter, float accelUp, float dt);
eSTATUS_t AltitudeFilter_Correct (AltitudeFilter_t* pFilter, float baroAlt, float dt);

// clang-format on

#endif // MOTION_CONTROL_FILTER_H
