/*
 * Altitude estimator: the ISA pressure conversion and the vertical
 * complementary filter, on the host.
 *
 * The pressure half has an ANALYTIC oracle. SensorSilResearch.md 2.3 measured
 * JSBSim's atmosphere against the inverse barometric formula from 0 to 1000 m
 * and found them within 5 cm, so the numbers below are real pressures at known
 * altitudes and the assertion is against the ISA rather than against a flight
 * model nobody has validated. That is what lets the tolerance be tight.
 *
 * The filter half is checked for the three things a complementary filter can
 * plausibly get wrong: it must converge to the measurement, it must integrate
 * acceleration correctly in between, and its bias state must absorb a constant
 * accelerometer offset instead of letting it become a standing climb rate.
 */

#include "unity/unity.h"

#include "common/filter.h"

#include <math.h>

// Pressures JSBSim reported at these altitudes; see SensorSilResearch.md 2.3.
#define ISA_SEA_LEVEL_PA 101325.0F

void setUp (void) {
}

void tearDown (void) {
}

static void ConfigureFilter (AltitudeFilter_t* pFilter) {
    // The CFG_ALT_FILTER_* values, restated rather than included: cfg.h drags
    // in a board target, and these gains are the subject of the test.
    pFilter->cfg.kAlt    = 3.0F;
    pFilter->cfg.kVel    = 3.0F;
    pFilter->cfg.kBias   = 1.0F;
    pFilter->cfg.maxBias = 2.0F;
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, AltitudeFilter_Init (pFilter));
}

/*
 * Run the filter for a number of seconds with a constant true altitude and a
 * constant measured acceleration: 400 Hz predict, 50 Hz correct, exactly the
 * rate ratio Nav_Update produces in the SIL.
 */
static void RunFilter (AltitudeFilter_t* pFilter, float baroAlt, float accelUp, float seconds) {
    const float dtImu   = 1.0F / 400.0F;
    const float dtBaro  = 1.0F / 50.0F;
    const int   steps   = (int)(seconds * 400.0F);

    for (int i = 0; i < steps; ++i) {
        AltitudeFilter_Predict (pFilter, accelUp, dtImu);
        if ((i % 8) == 7) {
            AltitudeFilter_Correct (pFilter, baroAlt, dtBaro);
        }
    }
}

void test_PressureToAltitude_MatchesIsaTable (void) {
    // Measured pressures, and the altitude each was measured at.
    TEST_ASSERT_FLOAT_WITHIN (0.1F, 0.0F, Baro_PressureToAltitude (101325.55F, ISA_SEA_LEVEL_PA));
    TEST_ASSERT_FLOAT_WITHIN (0.1F, 50.0F, Baro_PressureToAltitude (100726.33F, ISA_SEA_LEVEL_PA));
    TEST_ASSERT_FLOAT_WITHIN (0.1F, 100.0F, Baro_PressureToAltitude (100129.97F, ISA_SEA_LEVEL_PA));
    TEST_ASSERT_FLOAT_WITHIN (0.1F, 200.0F, Baro_PressureToAltitude (98945.93F, ISA_SEA_LEVEL_PA));
    TEST_ASSERT_FLOAT_WITHIN (0.1F, 500.0F, Baro_PressureToAltitude (95461.78F, ISA_SEA_LEVEL_PA));
    TEST_ASSERT_FLOAT_WITHIN (0.1F, 1000.0F, Baro_PressureToAltitude (89876.73F, ISA_SEA_LEVEL_PA));
}

/*
 * The datum is the whole point of the second argument: pressure equal to the
 * reference must read exactly zero, whatever that pressure is. This is what
 * makes nav.alt "height above where the board booted" rather than MSL.
 */
void test_PressureToAltitude_IsRelativeToDatum (void) {
    TEST_ASSERT_FLOAT_WITHIN (0.001F, 0.0F, Baro_PressureToAltitude (95461.78F, 95461.78F));
    TEST_ASSERT_FLOAT_WITHIN (0.001F, 0.0F, Baro_PressureToAltitude (101325.0F, 101325.0F));
}

/*
 * Referencing the sea-level formula to a datum that is NOT at sea level leaves
 * a systematic SCALE error, and this test pins its size rather than pretending
 * it is absent. Datum at 500 m, sample from 1000 m: the true height above the
 * datum is 500 m and the formula reads 505.7, i.e. +1.14%.
 *
 * Measured across the range (Scripts are in the commit notes): the error is
 * about +0.23% of height per 1000 m of DATUM elevation, and is exactly zero for
 * a sea-level datum. So a 100 m climb reads +0.23 m high from a 100 m field,
 * +1.14 m from a 500 m field, +2.31 m from 1000 m.
 *
 * That is accepted, not overlooked. Removing it means using the measured
 * temperature at the datum instead of the ISA's assumed 288.15 K - the baro
 * message already carries temperature, so it is available if wanted. It is not
 * done because every consumer of this number is RELATIVE (hold this height,
 * climb at this rate), and a sub-percent scale error is invisible to all of
 * them. Revisit it if absolute AGL ever has to agree with a survey.
 */
void test_PressureToAltitude_DatumScaleErrorIsBounded (void) {
    float relative = Baro_PressureToAltitude (89876.73F, 95461.78F);
    TEST_ASSERT_FLOAT_WITHIN (0.1F, 505.7F, relative);

    // Still monotonic and still the right order of magnitude - the error is a
    // scale factor, never a sign flip or a discontinuity.
    TEST_ASSERT_TRUE (relative > 500.0F);
    TEST_ASSERT_TRUE (relative < 510.0F);
}

void test_PressureToAltitude_RejectsBadInput (void) {
    TEST_ASSERT_EQUAL_FLOAT (0.0F, Baro_PressureToAltitude (0.0F, ISA_SEA_LEVEL_PA));
    TEST_ASSERT_EQUAL_FLOAT (0.0F, Baro_PressureToAltitude (-1.0F, ISA_SEA_LEVEL_PA));
    TEST_ASSERT_EQUAL_FLOAT (0.0F, Baro_PressureToAltitude (ISA_SEA_LEVEL_PA, 0.0F));
}

void test_AltitudeFilter_InitRejectsBadConfig (void) {
    AltitudeFilter_t filter = { 0 };

    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, AltitudeFilter_Init (NULL));

    // maxBias of zero would clamp the bias estimate permanently to zero, which
    // silently removes the third state rather than failing.
    filter.cfg.maxBias = 0.0F;
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, AltitudeFilter_Init (&filter));

    filter.cfg.maxBias = 2.0F;
    filter.cfg.kAlt    = -1.0F;
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, AltitudeFilter_Init (&filter));
}

void test_AltitudeFilter_RejectsNonPositiveDt (void) {
    AltitudeFilter_t filter = { 0 };
    ConfigureFilter (&filter);

    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, AltitudeFilter_Predict (&filter, 0.0F, 0.0F));
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, AltitudeFilter_Predict (&filter, 0.0F, -0.01F));
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, AltitudeFilter_Correct (&filter, 0.0F, 0.0F));
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, AltitudeFilter_Predict (NULL, 0.0F, 0.01F));
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, AltitudeFilter_Correct (NULL, 0.0F, 0.01F));

    // A rejected call must not have moved the state.
    TEST_ASSERT_EQUAL_FLOAT (0.0F, filter.alt);
    TEST_ASSERT_EQUAL_FLOAT (0.0F, filter.vz);
}

/*
 * Pure dead reckoning: no corrections at all, constant 1 m/s^2 up for one
 * second. This is the accel path on its own - if the 0.5*a*dt^2 term were
 * missing, alt would come out low by half a sample's worth every step.
 */
void test_AltitudeFilter_PredictIntegratesAcceleration (void) {
    AltitudeFilter_t filter = { 0 };
    ConfigureFilter (&filter);

    const float dt = 1.0F / 400.0F;
    for (int i = 0; i < 400; ++i) {
        TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, AltitudeFilter_Predict (&filter, 1.0F, dt));
    }

    TEST_ASSERT_FLOAT_WITHIN (0.001F, 1.0F, filter.vz);
    TEST_ASSERT_FLOAT_WITHIN (0.001F, 0.5F, filter.alt);
}

/*
 * Convergence: started at zero and shown a steady 10 m baro with no measured
 * acceleration, the estimate must reach the measurement and stop there.
 *
 * Ten seconds, not the five a first-order intuition suggests. The shipped gains
 * are the standard third-order set for T = 1 s, which makes the characteristic
 * polynomial s^3 + 3s^2 + 3s + 1 = (s+1)^3 - a TRIPLE pole at -1, whose step
 * response carries a t^2 term and so needs roughly 10 tau rather than 5 to
 * settle. Measured: 2.5% error at 5 s, 0.6% at 8 s, 0.15% at 10 s.
 *
 * That slowness is not a problem in flight, and it is worth being clear why:
 * this pole governs only how fast a baro-vs-inertial DISAGREEMENT is reconciled.
 * Real vehicle motion arrives through the accel path at 400 Hz and appears
 * immediately (see test_AltitudeFilter_TracksSteadyClimb), and at boot there is
 * no step at all because the datum defines the current height as zero. A step
 * this size only happens if the pressure datum was wrong, and reconciling that
 * slowly is the desired behaviour rather than a cost.
 */
void test_AltitudeFilter_ConvergesToBaro (void) {
    AltitudeFilter_t filter = { 0 };
    ConfigureFilter (&filter);

    RunFilter (&filter, 10.0F, 0.0F, 10.0F);

    TEST_ASSERT_FLOAT_WITHIN (0.05F, 10.0F, filter.alt);
    TEST_ASSERT_FLOAT_WITHIN (0.05F, 0.0F, filter.vz);

    // Converged, not merely passing through: run on and it must stay.
    RunFilter (&filter, 10.0F, 0.0F, 5.0F);
    TEST_ASSERT_FLOAT_WITHIN (0.01F, 10.0F, filter.alt);
    TEST_ASSERT_FLOAT_WITHIN (0.01F, 0.0F, filter.vz);
}

/*
 * The reason the third state exists. A constant 0.5 m/s^2 accelerometer offset
 * on a vehicle that is not moving must end up in accelBias, NOT in vz - an
 * uncorrected offset that size integrates to 0.5 m/s of phantom climb rate per
 * second, which a height controller would chase forever.
 */
void test_AltitudeFilter_AbsorbsConstantAccelBias (void) {
    AltitudeFilter_t filter = { 0 };
    ConfigureFilter (&filter);

    RunFilter (&filter, 0.0F, 0.5F, 30.0F);

    TEST_ASSERT_FLOAT_WITHIN (0.05F, 0.5F, filter.accelBias);
    TEST_ASSERT_FLOAT_WITHIN (0.05F, 0.0F, filter.vz);
    TEST_ASSERT_FLOAT_WITHIN (0.05F, 0.0F, filter.alt);
}

/*
 * The clamp is the guard against a wrong datum or a dead part turning the bias
 * integrator into a divergent one. A 10 m/s^2 offset is not a sensor that can
 * be trusted, and the estimate must stop at maxBias rather than run.
 */
void test_AltitudeFilter_ClampsRunawayBias (void) {
    AltitudeFilter_t filter = { 0 };
    ConfigureFilter (&filter);

    RunFilter (&filter, 0.0F, 10.0F, 60.0F);

    TEST_ASSERT_FLOAT_WITHIN (0.001F, filter.cfg.maxBias, filter.accelBias);
    TEST_ASSERT_TRUE (isfinite (filter.alt));
    TEST_ASSERT_TRUE (isfinite (filter.vz));
}

/*
 * A climb the accel knows about before the baro does. Ramping the baro at
 * 2 m/s while feeding the matching (zero, since the climb is steady) vertical
 * acceleration must produce a velocity estimate that tracks the ramp rather
 * than lagging a full time constant behind it.
 */
void test_AltitudeFilter_TracksSteadyClimb (void) {
    AltitudeFilter_t filter = { 0 };
    ConfigureFilter (&filter);

    const float dtImu  = 1.0F / 400.0F;
    const float dtBaro = 1.0F / 50.0F;
    const float rate   = 2.0F;
    float trueAlt      = 0.0F;

    for (int i = 0; i < 4000; ++i) {   // 10 s
        trueAlt += rate * dtImu;
        AltitudeFilter_Predict (&filter, 0.0F, dtImu);
        if ((i % 8) == 7) {
            AltitudeFilter_Correct (&filter, trueAlt, dtBaro);
        }
    }

    TEST_ASSERT_FLOAT_WITHIN (0.1F, rate, filter.vz);
    TEST_ASSERT_FLOAT_WITHIN (0.5F, trueAlt, filter.alt);
}

int main (void) {
    UNITY_BEGIN ();
    RUN_TEST (test_PressureToAltitude_MatchesIsaTable);
    RUN_TEST (test_PressureToAltitude_IsRelativeToDatum);
    RUN_TEST (test_PressureToAltitude_DatumScaleErrorIsBounded);
    RUN_TEST (test_PressureToAltitude_RejectsBadInput);
    RUN_TEST (test_AltitudeFilter_InitRejectsBadConfig);
    RUN_TEST (test_AltitudeFilter_RejectsNonPositiveDt);
    RUN_TEST (test_AltitudeFilter_PredictIntegratesAcceleration);
    RUN_TEST (test_AltitudeFilter_ConvergesToBaro);
    RUN_TEST (test_AltitudeFilter_AbsorbsConstantAccelBias);
    RUN_TEST (test_AltitudeFilter_ClampsRunawayBias);
    RUN_TEST (test_AltitudeFilter_TracksSteadyClimb);
    return UNITY_END ();
}
