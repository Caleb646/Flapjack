/*
 * Horizontal estimator: the GPS/accel complementary filter, on the host.
 *
 * Checked for the same three things the altitude test checks its vertical twin
 * for - it must converge to the measurement, it must integrate acceleration
 * correctly in between, and its bias state must absorb a standing accelerometer
 * offset instead of letting it become a permanent velocity error - plus the one
 * property that is specific to this axis pair and is the reason the filter
 * exists at all.
 *
 * THAT PROPERTY: the bias state has to converge on the SPURIOUS part of the
 * horizontal specific force, so that subtracting it leaves the acceleration the
 * vehicle is really doing. nav.c hands that difference to the attitude filter
 * as the translational correction for KnownIssues 1.20, and it is only worth
 * anything if the two cases come out different:
 *
 *   really accelerating   GPS agrees, bias stays near zero, correction = accel
 *   tilt error only       GPS disagrees, bias absorbs it,   correction = zero
 *
 * Getting that backwards still passes a convergence test and still flies - it
 * just reinstates 1.20 - so it is asserted directly.
 */

#include "unity/unity.h"

#include "common/filter.h"

#include <math.h>

// Gains as nav.c configures them, so the test exercises the shipped tune.
#define TEST_K_POS    0.5F
#define TEST_K_VEL    1.0F
#define TEST_K_BIAS   0.25F
#define TEST_MAX_BIAS 5.0F

#define TEST_IMU_DT 0.0025F   // 400 Hz, the rate Predict runs at
#define TEST_GPS_DT 0.1F      // 10 Hz, the rate Correct runs at

void setUp (void) {
}

void tearDown (void) {
}

static void Init (HorizontalFilter_t* pFilter) {
    pFilter->cfg.kPos    = TEST_K_POS;
    pFilter->cfg.kVel    = TEST_K_VEL;
    pFilter->cfg.kBias   = TEST_K_BIAS;
    pFilter->cfg.maxBias = TEST_MAX_BIAS;
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, HorizontalFilter_Init (pFilter));
}

/*
 * Fly `seconds` with the IMU reporting `accelNed` and the receiver reporting
 * `gpsPos`/`gpsVel`, at their real relative rates. The GPS arrives every
 * TEST_GPS_DT, the IMU every TEST_IMU_DT, which is what makes the predict and
 * correct gains mean what they say.
 */
static void Fly (HorizontalFilter_t* pFilter, float const* accelNed,
                 float const* gpsPos, float const* gpsVel, float seconds) {
    int steps = (int)(seconds / TEST_IMU_DT);
    int perGps = (int)(TEST_GPS_DT / TEST_IMU_DT);
    for (int i = 1; i <= steps; i++) {
        TEST_ASSERT_EQUAL (eSTATUS_SUCCESS,
                           HorizontalFilter_Predict (pFilter, accelNed, TEST_IMU_DT));
        if ((i % perGps) == 0) {
            TEST_ASSERT_EQUAL (eSTATUS_SUCCESS,
                               HorizontalFilter_Correct (pFilter, gpsPos, gpsVel, TEST_GPS_DT));
        }
    }
}

void test_HorizontalFilter_InitRejectsBadConfig (void) {
    HorizontalFilter_t filter;
    Init (&filter);

    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, HorizontalFilter_Init (NULL));

    filter.cfg.maxBias = 0.0F;   // a zero clamp would pin the bias at zero
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, HorizontalFilter_Init (&filter));

    Init (&filter);
    filter.cfg.kVel = -1.0F;
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, HorizontalFilter_Init (&filter));
}

void test_HorizontalFilter_RejectsNonPositiveDt (void) {
    HorizontalFilter_t filter;
    Init (&filter);
    float zero[HORIZ_AXES] = { 0.0F, 0.0F };

    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, HorizontalFilter_Predict (&filter, zero, 0.0F));
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, HorizontalFilter_Predict (&filter, NULL, TEST_IMU_DT));
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, HorizontalFilter_Correct (&filter, zero, zero, -1.0F));
}

// Dead reckoning with no corrections at all: v = a*t, p = 0.5*a*t^2, per axis.
void test_HorizontalFilter_PredictIntegratesAcceleration (void) {
    HorizontalFilter_t filter;
    Init (&filter);
    float accel[HORIZ_AXES] = { 2.0F, -1.0F };

    for (int i = 0; i < 400; i++) {   // 1 s
        HorizontalFilter_Predict (&filter, accel, TEST_IMU_DT);
    }

    TEST_ASSERT_FLOAT_WITHIN (0.02F, 2.0F, filter.vel[0]);
    TEST_ASSERT_FLOAT_WITHIN (0.02F, -1.0F, filter.vel[1]);
    TEST_ASSERT_FLOAT_WITHIN (0.02F, 1.0F, filter.pos[0]);
    TEST_ASSERT_FLOAT_WITHIN (0.02F, -0.5F, filter.pos[1]);
}

// A stationary vehicle whose receiver says it is 10 m north and 5 m east must
// end up there, on both axes independently.
void test_HorizontalFilter_ConvergesToGps (void) {
    HorizontalFilter_t filter;
    Init (&filter);
    float accel[HORIZ_AXES] = { 0.0F, 0.0F };
    float pos[HORIZ_AXES]   = { 10.0F, 5.0F };
    float vel[HORIZ_AXES]   = { 0.0F, 0.0F };

    Fly (&filter, accel, pos, vel, 30.0F);

    TEST_ASSERT_FLOAT_WITHIN (0.5F, 10.0F, filter.pos[0]);
    TEST_ASSERT_FLOAT_WITHIN (0.5F, 5.0F, filter.pos[1]);
    TEST_ASSERT_FLOAT_WITHIN (0.1F, 0.0F, filter.vel[0]);
    TEST_ASSERT_FLOAT_WITHIN (0.1F, 0.0F, filter.vel[1]);
}

/*
 * A standing accelerometer offset the vehicle is NOT experiencing must land in
 * the bias state, not in the velocity. Without the bias state the velocity
 * error is whatever the correction gain happens to balance the offset at, and
 * it never goes away.
 */
void test_HorizontalFilter_AbsorbsAccelBias (void) {
    HorizontalFilter_t filter;
    Init (&filter);
    float offset[HORIZ_AXES] = { 0.4F, -0.3F };   // m/s^2 the IMU claims
    float pos[HORIZ_AXES]    = { 0.0F, 0.0F };    // receiver says: going nowhere
    float vel[HORIZ_AXES]    = { 0.0F, 0.0F };

    Fly (&filter, offset, pos, vel, 40.0F);

    TEST_ASSERT_FLOAT_WITHIN (0.05F, 0.4F, filter.accelBias[0]);
    TEST_ASSERT_FLOAT_WITHIN (0.05F, -0.3F, filter.accelBias[1]);
    TEST_ASSERT_FLOAT_WITHIN (0.1F, 0.0F, filter.vel[0]);
    TEST_ASSERT_FLOAT_WITHIN (0.1F, 0.0F, filter.vel[1]);
}

void test_HorizontalFilter_ClampsRunawayBias (void) {
    HorizontalFilter_t filter;
    Init (&filter);
    float absurd[HORIZ_AXES] = { 50.0F, -50.0F };   // a broken part, or a glitch
    float zero[HORIZ_AXES]   = { 0.0F, 0.0F };

    Fly (&filter, absurd, zero, zero, 60.0F);

    TEST_ASSERT_TRUE (fabsf (filter.accelBias[0]) <= TEST_MAX_BIAS + 0.001F);
    TEST_ASSERT_TRUE (fabsf (filter.accelBias[1]) <= TEST_MAX_BIAS + 0.001F);
}

/*
 * The property nav.c actually consumes, both halves of it.
 *
 * The correction it applies is (accelNed - accelBias). When the vehicle really
 * is accelerating and the receiver agrees, that must come out AS the
 * acceleration - there is a real translation to remove from the gravity
 * reference. When the IMU reports the same acceleration but the receiver says
 * the vehicle is not moving - which is what a tilt error looks like, since it
 * leaks g*sin(theta) into the rotated accel - it must come out near ZERO,
 * because there is no translation to remove and subtracting one would tell the
 * attitude filter it is already level and freeze the error.
 */
void test_HorizontalFilter_BiasSeparatesRealAccelFromTiltError (void) {
    float accel[HORIZ_AXES] = { 1.5F, 0.0F };

    // Case 1: genuinely accelerating north at 1.5 m/s^2, receiver agrees.
    HorizontalFilter_t moving;
    Init (&moving);
    for (int i = 1; i <= 4000; i++) {   // 10 s
        HorizontalFilter_Predict (&moving, accel, TEST_IMU_DT);
        if ((i % 40) == 0) {
            float t                 = i * TEST_IMU_DT;
            float pos[HORIZ_AXES]   = { 0.5F * 1.5F * t * t, 0.0F };
            float vel[HORIZ_AXES]   = { 1.5F * t, 0.0F };
            HorizontalFilter_Correct (&moving, pos, vel, TEST_GPS_DT);
        }
    }
    float correctionMoving = accel[0] - moving.accelBias[0];

    // Case 2: same reading, receiver says the vehicle is stationary.
    HorizontalFilter_t tilted;
    Init (&tilted);
    float still[HORIZ_AXES] = { 0.0F, 0.0F };
    Fly (&tilted, accel, still, still, 10.0F);
    float correctionTilted = accel[0] - tilted.accelBias[0];

    // Real acceleration survives the subtraction.
    TEST_ASSERT_FLOAT_WITHIN (0.2F, 1.5F, correctionMoving);
    // A tilt error does not.
    TEST_ASSERT_FLOAT_WITHIN (0.2F, 0.0F, correctionTilted);
}

int main (void) {
    UNITY_BEGIN ();
    RUN_TEST (test_HorizontalFilter_InitRejectsBadConfig);
    RUN_TEST (test_HorizontalFilter_RejectsNonPositiveDt);
    RUN_TEST (test_HorizontalFilter_PredictIntegratesAcceleration);
    RUN_TEST (test_HorizontalFilter_ConvergesToGps);
    RUN_TEST (test_HorizontalFilter_AbsorbsAccelBias);
    RUN_TEST (test_HorizontalFilter_ClampsRunawayBias);
    RUN_TEST (test_HorizontalFilter_BiasSeparatesRealAccelFromTiltError);
    return UNITY_END ();
}
