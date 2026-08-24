/*
 * Madgwick attitude filter: the accelerometer trust knob, on the host.
 *
 * MadgwickFilter_t.accelTrust is how nav.c tells the filter that the
 * accelerometer may not be a gravity reference this iteration - it holds when
 * there is no GPS velocity to subtract the vehicle's own translation with
 * (KnownIssues 1.20, CFG_NAV_ACCEL_TRUST_UNAIDED).
 *
 * It is pinned here rather than in the SIL for two reasons. The SIL cannot
 * reach the path at all - every settle-probe run has a GPS fix throughout, so
 * accelTrust is 1.0 for the whole flight - and the property that matters is a
 * SPLIT that no end-to-end number would expose: the knob must slow the
 * accelerometer WITHOUT slowing the magnetometer, and filter.c has to reach
 * that two different ways in its two paths. 6DOF scales beta; 9DOF scales the
 * accel rows of the objective function and leaves beta alone, because beta is
 * one gain over a combined gradient there. Get the paths backwards and 6DOF
 * silently does nothing at all (normalising a gradient built only from the
 * accel rows divides any common factor straight back out) while 9DOF drags
 * heading down with roll and pitch. Both mistakes fly.
 */

#include "unity/unity.h"

#include "common/filter.h"

#include <math.h>

// Level and still in this project's convention - see devices/imu.c.
#define ACCEL_LEVEL_X 0.0F
#define ACCEL_LEVEL_Y 0.0F
#define ACCEL_LEVEL_Z 9.80665F

// Deliberately large so a single step moves the estimate measurably.
#define TEST_GYRO_ERR_DEGS 90.0F
#define TEST_STEP_DT       0.0025F   // 400 Hz, the IMU rate nav.c runs at
#define TEST_STEPS         400       // one second of them

void setUp (void) {
}

void tearDown (void) {
}

static void Init (MadgwickFilter_t* pFilter, float trust) {
    pFilter->cfg.gyroMeasureErrorDegs = TEST_GYRO_ERR_DEGS;
    pFilter->cfg.gyroMeasureDriftDegs = 0.0F;
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, MadgwickFilter_Init (pFilter));
    pFilter->accelTrust = trust;
}

/* An accel tilted in roll, at 1 g so MadgwickFilter_AccelUsable stays out of
 * the way - the trust knob is the only thing under test here. */
static void TiltedAccel (Vec3f* pAccel, float rollDeg) {
    float r     = rollDeg * (float)M_PI / 180.0F;
    pAccel->x = 0.0F;
    /* Rolled right by phi, the specific force in this project's convention is
     * g * (0, sin phi, cos phi) - the bottom row of the body->NED matrix times
     * gravity. Getting the y sign wrong here converges to -phi and looks like
     * a filter bug. */
    pAccel->y = 9.80665F * sinf (r);
    pAccel->z = 9.80665F * cosf (r);
}

/*
 * Accel AND mag as a real pair of parts would read them at a given attitude:
 * both are the earth-frame vector rotated into body, so they agree with each
 * other about where the vehicle is pointing.
 *
 * Synthesising them together is the only way to make a 9DOF test mean anything.
 * A hand-written mag vector will not generally be consistent with the accel,
 * and an inconsistent pair has no attitude to converge to - a horizontal field
 * (zero vertical component, which no mid-latitude site has) sent this filter to
 * 180 deg of roll at FULL trust while it was being asked about a 20 deg one.
 *
 * Field is the SIL's own reference: 0.5 G at 60 deg inclination, zero
 * declination (bridge.py B_NED), so the two rigs cannot drift apart on it.
 */
#define TEST_FIELD_GAUSS 0.50F
#define TEST_INCL_DEG    60.0F

static void SynthSensors (float rollDeg, float yawDeg, Vec3f* pAccel, Vec3f* pMag) {
    float r  = rollDeg * (float)M_PI / 180.0F;
    float y  = yawDeg * (float)M_PI / 180.0F;
    float in = TEST_INCL_DEG * (float)M_PI / 180.0F;

    float bN = TEST_FIELD_GAUSS * cosf (in);
    float bD = TEST_FIELD_GAUSS * sinf (in);

    /* R is body->NED for roll r, pitch 0, yaw y; both readings are R^T times
     * the earth-frame vector, which for a unit column is just that column of R
     * read down the rows. */
    float cr = cosf (r), sr = sinf (r);
    float cy = cosf (y), sy = sinf (y);

    // R^T * (0, 0, g)  -  the bottom row of R, as in TiltedAccel above.
    pAccel->x = 0.0F;
    pAccel->y = 9.80665F * sr;
    pAccel->z = 9.80665F * cr;

    /* R^T * (bN, 0, bD), with R = Rz(yaw) * Rx(roll) and pitch zero. */
    pMag->x = cy * bN;
    pMag->y = (-sy * cr * bN) + (sr * bD);
    pMag->z = (sy * sr * bN) + (cr * bD);
}

static void RunSteps (MadgwickFilter_t* pFilter, Vec3f const* pAccel, Vec3f const* pMag, int steps) {
    Vec3f gyro  = { 0.0F, 0.0F, 0.0F };
    Vec3f euler = { 0.0F, 0.0F, 0.0F };
    for (int i = 0; i < steps; i++) {
        TEST_ASSERT_EQUAL (eSTATUS_SUCCESS,
                           MadgwickFilter_Update (pFilter, pAccel, &gyro, pMag, TEST_STEP_DT, &euler));
    }
}

static void EulerAfterTilt (float trust, Vec3f const* pMag, int steps, Vec3f* pOutEuler) {
    MadgwickFilter_t filter;
    Init (&filter, trust);
    Vec3f accel;
    TiltedAccel (&accel, 20.0F);
    RunSteps (&filter, &accel, pMag, steps);
    MadgwickFilter_QuatToEuler (&filter, pOutEuler);
}

static float RollAfterTilt (float trust, Vec3f const* pMag) {
    Vec3f euler;
    EulerAfterTilt (trust, pMag, TEST_STEPS, &euler);
    return euler.x;
}

// Init must leave the filter exactly as it behaved before the knob existed.
void test_AccelTrust_DefaultsToUnity (void) {
    MadgwickFilter_t filter;
    filter.cfg.gyroMeasureErrorDegs = TEST_GYRO_ERR_DEGS;
    filter.cfg.gyroMeasureDriftDegs = 0.0F;
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, MadgwickFilter_Init (&filter));
    TEST_ASSERT_EQUAL_FLOAT (1.0F, filter.accelTrust);
}

// 6DOF: full trust must chase the tilted accel.
void test_AccelTrust_6DOF_FullTrustFollowsAccel (void) {
    TEST_ASSERT_FLOAT_WITHIN (3.0F, 20.0F, RollAfterTilt (1.0F, NULL));
}

// 6DOF: zero trust must leave the estimate exactly where the gyro put it. With
// no magnetometer there is nothing else to apply, so the estimate must not move.
void test_AccelTrust_6DOF_ZeroTrustIgnoresAccel (void) {
    TEST_ASSERT_FLOAT_WITHIN (0.001F, 0.0F, RollAfterTilt (0.0F, NULL));
}

/*
 * 6DOF: the knob has to be monotonic to be a knob at all.
 *
 * Measured PART WAY THROUGH the convergence, not at the end. The knob scales a
 * rate, so given long enough every non-zero trust reaches the same answer and
 * the comparison below becomes 20 == 20. TEST_SETTLE_STEPS is a second, which
 * is long enough for that; a tenth of one is not.
 */
#define TEST_PARTIAL_STEPS 40

void test_AccelTrust_6DOF_IsMonotonic (void) {
    Vec3f full, half, none;
    EulerAfterTilt (1.0F, NULL, TEST_PARTIAL_STEPS, &full);
    EulerAfterTilt (0.5F, NULL, TEST_PARTIAL_STEPS, &half);
    EulerAfterTilt (0.0F, NULL, TEST_PARTIAL_STEPS, &none);
    TEST_ASSERT_TRUE (none.x < half.x);
    TEST_ASSERT_TRUE (half.x < full.x);
}

/*
 * 9DOF, the split this whole file exists for: reduced trust must slow the
 * ACCELEROMETER without slowing the MAGNETOMETER.
 *
 * Truth is 20 deg of roll and 30 deg of heading, and the filter starts level
 * and pointing north, so both halves have somewhere to go and one sensor drives
 * each. Both are measured against a full-trust run of the same length, which is
 * what makes the claim falsifiable in both directions at once - roll must come
 * out lower, heading must not. Scaling beta instead of the accel rows passes
 * the roll half and fails the heading half.
 *
 * Trust is CFG_NAV_ACCEL_TRUST_UNAIDED, the value nav.c actually sets, not 0.
 * Zero is a different case, not a stricter one: with the accel rows gone
 * entirely the objective is only the mag rows, which are invariant under
 * rotation about the field vector, so the descent is free to walk that circle.
 * That hazard is NOT introduced by this knob - MadgwickFilter_AccelUsable
 * already zeroes the same three rows when the accel magnitude leaves its band -
 * but it is why this test does not use 0 for a cleaner assertion.
 */
#define TEST_TRUST_UNAIDED 0.5F

static void EulerAt (float trust, int steps, Vec3f* pOutEuler) {
    MadgwickFilter_t filter;
    Init (&filter, trust);
    Vec3f accel, mag;
    SynthSensors (20.0F, 30.0F, &accel, &mag);
    RunSteps (&filter, &accel, &mag, steps);
    MadgwickFilter_QuatToEuler (&filter, pOutEuler);
}

void test_AccelTrust_9DOF_SlowsAccelNotHeading (void) {
    Vec3f full, reduced;
    EulerAt (1.0F, TEST_PARTIAL_STEPS, &full);
    EulerAt (TEST_TRUST_UNAIDED, TEST_PARTIAL_STEPS, &reduced);

    // Both runs are still on their way, or there is nothing to compare.
    TEST_ASSERT_TRUE (full.x > 1.0F);
    TEST_ASSERT_TRUE (fabsf (full.z) > 1.0F);

    // The accelerometer was slowed: less roll than at full trust.
    TEST_ASSERT_TRUE (reduced.x < full.x);

    /*
     * The magnetometer was NOT slowed. Not "was unchanged" - it speeds up
     * slightly, and that is a real property rather than slop in a tolerance.
     * Madgwick normalises the gradient before scaling it by beta, so the step
     * is always beta in SOME direction; shrinking the accel rows leaves the mag
     * rows a larger share of that fixed step.
     *
     * Hence a one-sided assertion, and it is the one that catches the mistake
     * this file exists for: scaling beta instead of the accel rows slows BOTH,
     * so heading would come out under the full-trust value.
     */
    TEST_ASSERT_TRUE (fabsf (reduced.z) >= fabsf (full.z));
}

// 9DOF with a consistent sensor pair must converge on the attitude that
// produced it - the baseline the test above compares against.
void test_AccelTrust_9DOF_FullTrustConverges (void) {
    Vec3f euler;
    EulerAt (1.0F, TEST_STEPS, &euler);
    TEST_ASSERT_FLOAT_WITHIN (2.0F, 20.0F, euler.x);
    TEST_ASSERT_FLOAT_WITHIN (2.0F, 0.0F, euler.y);
    TEST_ASSERT_FLOAT_WITHIN (3.0F, 30.0F, euler.z);
}

/*
 * The magnitude gate still wins over the knob. A reading that cannot be gravity
 * must be dropped whatever the caller asked for - trust scales a plausible
 * reading, it does not license an implausible one.
 */
void test_AccelTrust_DoesNotOverrideMagnitudeGate (void) {
    MadgwickFilter_t filter;
    Init (&filter, 1.0F);
    Vec3f accel = { 0.0F, -9.80665F * 2.0F, 0.0F };   // 2 g sideways
    RunSteps (&filter, &accel, NULL, TEST_STEPS);
    Vec3f euler;
    MadgwickFilter_QuatToEuler (&filter, &euler);
    TEST_ASSERT_FLOAT_WITHIN (0.001F, 0.0F, euler.x);
}

int main (void) {
    UNITY_BEGIN ();
    RUN_TEST (test_AccelTrust_DefaultsToUnity);
    RUN_TEST (test_AccelTrust_6DOF_FullTrustFollowsAccel);
    RUN_TEST (test_AccelTrust_6DOF_ZeroTrustIgnoresAccel);
    RUN_TEST (test_AccelTrust_6DOF_IsMonotonic);
    RUN_TEST (test_AccelTrust_9DOF_SlowsAccelNotHeading);
    RUN_TEST (test_AccelTrust_9DOF_FullTrustConverges);
    RUN_TEST (test_AccelTrust_DoesNotOverrideMagnitudeGate);
    return UNITY_END ();
}
