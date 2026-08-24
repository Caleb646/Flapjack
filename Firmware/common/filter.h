#ifndef MOTION_CONTROL_FILTER_H
#define MOTION_CONTROL_FILTER_H

#include "core/core.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * First-order (PT1) low-pass, parameterised by CUTOFF rather than by a fixed
 * coefficient.
 *
 * The obvious form - y += alpha * (x - y) with a constant alpha, which is what
 * this struct used to hold - pins the corner to the SAMPLE RATE:
 * f_c = alpha / (2*pi*dt*(1 - alpha)). Every loop here is sensor-paced rather
 * than fixed; the rate PID follows the IMU and the same build has been measured
 * between 250 and 366 Hz depending only on host load in the SIL. A constant
 * alpha would let the corner wander with that, which is the same class of bug
 * the CONTROL_RATE_HZ static assert in control.c exists to prevent. Taking dt
 * per call and deriving alpha from it keeps the corner where it was configured.
 *
 * cutoffHz <= 0 disables the filter and Update passes its input straight
 * through, so a configured 0 means "no filtering" rather than "filter
 * everything away".
 */
typedef struct {
    struct {
        float cutoffHz; // -3 dB corner, Hz. <= 0 disables (pass-through).
    } cfg;
    float state;
    bool  hasState;
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

/*
 * Horizontal estimator: the same third-order complementary filter as
 * AltitudeFilter, one axis each for North and East, fusing GPS with the
 * horizontal acceleration recovered from the IMU.
 *
 * Same shape and the same reasoning as the vertical channel next door. GPS is
 * quiet in the long run but arrives at ~10 Hz and carries metres of position
 * noise; integrated accel is smooth and instantaneous but drifts without
 * bound. So predict from accel at the IMU rate and correct towards GPS at the
 * receiver's rate, as two calls rather than one, because they are driven by
 * different sensors on different clocks.
 *
 * WHAT THE BIAS STATE IS FOR HERE, and it is not what the vertical one is for.
 * On that axis it absorbs a sensor offset. On these two it absorbs any standing
 * horizontal specific force the vehicle is not actually accelerating with -
 * which is dominated by ATTITUDE ERROR, not by the part. A tilt error of theta
 * leaks g*sin(theta) into the rotated horizontal accel, so 1 deg of it is
 * 0.17 m/s^2, and GPS is what says the vehicle is not really accelerating that
 * way. That is why maxBias is far larger than the vertical filter's: it has to
 * accommodate the tilt errors it exists to absorb, not just a healthy part.
 *
 * WHY THE CORRECTION TAKES BOTH POSITION AND VELOCITY, where the baro gives
 * only the one. The receiver reports both, and velocity is much the better
 * conditioned of the two - the SIL's own model puts 0.1 m/s on velocity against
 * metres on position (SensorGps.xml) - so velocity drives the bias, which is
 * the state that matters, and position only trims position. Differentiating the
 * position instead would hand the bias the noisiest signal available.
 *
 * Sign convention: NED, north and east positive, index 0 = North, 1 = East.
 * Down is the AltitudeFilter's problem, not this one's.
 */
#define HORIZ_AXES 2

typedef struct {
    struct {
        float kPos;    // position error -> position    [1/s]
        float kVel;    // velocity error -> velocity    [1/s]
        float kBias;   // velocity error -> accel bias  [1/s^2]
        float maxBias; // clamp on the bias estimate    [m/s^2]
    } cfg;
    float pos[HORIZ_AXES];       // metres from the origin, N then E
    float vel[HORIZ_AXES];       // metres/second, N then E
    float accelBias[HORIZ_AXES]; // m/s^2, N then E
} HorizontalFilter_t;

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
    /*
     * How far the ACCELEROMETER is allowed to drag the estimate on the next
     * update. 1.0 is "as configured"; Init leaves it there, so a caller that
     * never touches it gets the unmodified filter.
     *
     * It exists because the caller sometimes knows something about the
     * accelerometer that this filter cannot see. nav.c can subtract the
     * vehicle's own translational acceleration when it has GPS velocity, and
     * when it cannot, the reading is a false vertical the filter would converge
     * on at full rate (KnownIssues 1.20).
     *
     * Accelerometer only, in BOTH paths - it must never slow the heading
     * correction. The two get there differently and filter.c says why at each
     * site: 9DOF scales the accel rows of the gradient, 6DOF has to scale beta,
     * because with no mag rows to weigh against, normalising the gradient
     * divides a common factor on the accel rows straight back out.
     */
    float accelTrust;
    // reference direction of flux in earth frame
    float bx;
    float bz;
    uint32_t usLastUpdateTime;
} MadgwickFilter_t;

// clang-format off

eSTATUS_t MadgwickFilter_Update (MadgwickFilter_t* pFilter, Vec3f const* pAccel, Vec3f const* pGyro, Vec3f const* pMag, float dt, Vec3f* pOutAttitude);
void MadgwickFilter_QuatToEuler (MadgwickFilter_t const* pFilter, Vec3f* pOutEuler);
eSTATUS_t MadgwickFilter_Init (MadgwickFilter_t* pFilter);

eSTATUS_t LowPassFilter_Init (LowPassFilter_t* pFilter);
float LowPassFilter_Update (LowPassFilter_t* pFilter, float input, float dt);

eSTATUS_t AltitudeFilter_Init (AltitudeFilter_t* pFilter);
eSTATUS_t AltitudeFilter_Predict (AltitudeFilter_t* pFilter, float accelUp, float dt);
eSTATUS_t AltitudeFilter_Correct (AltitudeFilter_t* pFilter, float baroAlt, float dt);

eSTATUS_t HorizontalFilter_Init (HorizontalFilter_t* pFilter);
eSTATUS_t HorizontalFilter_Predict (HorizontalFilter_t* pFilter, float const* accelNed, float dt);
eSTATUS_t HorizontalFilter_Correct (HorizontalFilter_t* pFilter, float const* posNed,
                                    float const* velNed, float dt);

// clang-format on

#endif // MOTION_CONTROL_FILTER_H
