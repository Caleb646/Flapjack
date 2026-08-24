#include "tasks/nav/nav.h"
#include "common/filter.h"
#include "devices/baro.h"
#include "umsg_sensors.h"
#include "umsg_nav.h"

#include "FreeRTOS.h"

#include <math.h>

/*
 * Baro samples averaged at startup to fix the pressure datum. At the sim link's
 * ~50 Hz that is one second - long enough to average down sample noise, short
 * enough that altitude is available well before the arming gate can open
 * (mission.c wants 3 s of held attitude first).
 *
 * The datum is taken at BOOT, not at arming. Arming is the better physical
 * reference - it is the moment the vehicle is known to be sitting on the
 * ground - but nav would have to subscribe to umsg_mission_state_t to see it,
 * and mission already subscribes to umsg_nav_state_t. That closes a dependency
 * cycle the layer rules forbid (Firmware/CLAUDE.md: no layer reads from a layer
 * above it). Boot costs nothing architecturally, and it makes nav.alt read
 * "height above wherever this was switched on", which is what a hover
 * controller wants anyway.
 */
#define NAV_BARO_DATUM_SAMPLES 50U

/*
 * WGS-84 mean radius, for the flat-earth projection of lat/lon onto NED.
 *
 * Flat-earth error grows with the square of the distance from the origin: it is
 * under a centimetre at 1 km and about a metre at 10 km. This vehicle does not
 * fly far enough for that to be the dominant error - the receiver's own
 * metre-class noise is - so the projection is not worth doing properly here.
 */
#define NAV_EARTH_RADIUS_M 6371000.0F

// Standard gravity, removed from the rotated specific-force vector.
#define NAV_GRAVITY_MPS2 9.80665F

/*
 * Below this specific-force magnitude the reading has no direction to compare
 * against, so the residual monitor reports maximal disagreement rather than a
 * number. 0.1 m/s^2 is far under anything a powered vehicle produces and well
 * over the part's noise floor, so it separates free fall from flight.
 */
#define NAV_ACCEL_MIN_MPS2 0.1F

/*
 * A 2D fix is enough. Altitude comes from the baro, not from GPS, so the third
 * dimension the receiver would add is one this estimator does not consume.
 */
#define NAV_GPS_MIN_FIX_TYPE 2U

typedef struct {
    MadgwickFilter_t filter;
    AltitudeFilter_t altFilter;
    HorizontalFilter_t horizFilter;

    umsg_sub_handle_t imu_sub;
    umsg_sub_handle_t mag_sub;
    umsg_sub_handle_t baro_sub;
    umsg_sub_handle_t gps_sub;

    uint32_t usLastUpdateTime;
    bool haveFirstSample;

    /* Pressure and temperature datum, averaged over the first
     * NAV_BARO_DATUM_SAMPLES samples. The temperature is part of the datum, not
     * incidental telemetry - Baro_PressureToAltitude scales the whole altitude
     * by it. */
    float datumSumPa;
    float datumSumTempC;
    uint32_t datumSamples;
    float refPressurePa;
    float refTempC;
    bool haveDatum;
    uint32_t usLastBaroTime;

    /* Flat-earth origin, captured once at the first usable fix. */
    double originLatDeg;
    double originLonDeg;
    float originCosLat;
    bool haveOrigin;
    bool haveFix;

    /* Last fix as decoded, NED metres and metres/second from the origin. These
     * are the horizontal filter's MEASUREMENT, not the estimate - the estimate
     * is horizFilter's own pos/vel, and that is what gets published. */
    float gpsPosNed[HORIZ_AXES];
    float gpsVelNed[HORIZ_AXES];
    uint32_t usLastFixTime;

    /* Translational acceleration differenced from the GPS velocity, and the
     * state that recovers it. Deliberately NOT taken from horizFilter - see
     * CFG_NAV_TRANS_ACCEL_CUTOFF_HZ for why the noisier open-loop source is
     * the one that can correct attitude. */
    LowPassFilter_t transAccelFiltN;
    LowPassFilter_t transAccelFiltE;
    float transAccelNorth;
    float transAccelEast;
    float lastVelNorth;
    float lastVelEast;
    uint32_t usLastVelTime;
    bool haveVelRate;
} Nav_t;

static Nav_t s_Nav;

/*
 * Vertical acceleration in m/s^2, UP positive, with gravity removed.
 *
 * The IMU reports specific force in the body frame, and this project's
 * convention - stated in sim.proto and echoed by the Madgwick filter's own
 * comment - is that level and still reads (0, 0, +9.81) in FRD. That is g - a,
 * the NEGATIVE of the physical proper acceleration, so rotating into NED and
 * subtracting g yields the UP-positive acceleration directly, with no further
 * sign flip:
 *
 *      level & still -> fDown = +9.81 -> 0
 *      free fall     -> fDown =  0    -> -9.81
 *      climbing hard -> fDown > +9.81 -> positive
 *
 * Only the "down" row of the body->NED rotation is needed, so the full matrix
 * is never formed. qEst is (w, x, y, z) - MadgwickFilter_QuatToEuler extracts
 * the standard aerospace Euler angles from it, which fixes it as the body->NED
 * quaternion, so this is that matrix's standard bottom row.
 */
STATIC inline float Nav_VerticalAccelUp (Vec4f const* pQuat, Vec3f const* pAccel) {

    float w = pQuat->q1;
    float x = pQuat->q2;
    float y = pQuat->q3;
    float z = pQuat->q4;

    float fDown = (2.0F * ((x * z) - (w * y)) * pAccel->x) +
                  (2.0F * ((y * z) + (w * x)) * pAccel->y) +
                  ((1.0F - (2.0F * ((x * x) + (y * y)))) * pAccel->z);

    return fDown - NAV_GRAVITY_MPS2;
}

/*
 * Whether the horizontal filter's bias estimate is fit to correct attitude
 * with.
 *
 * It is only worth anything once a receiver has argued with it. Before the
 * first fix the bias is zero and the "translational acceleration" it yields is
 * just the accelerometer read through the current attitude - so applying it
 * would cancel the horizontal part of the reading in the ESTIMATE's own frame
 * and tell the filter it is already level, freezing whatever error it has.
 * Self-confirming, and worse than no correction at all.
 *
 * The age bound is the same argument one step later: the filter keeps
 * integrating through a dropout, but a bias nothing has corrected for half a
 * second must stop rotating the gravity reference.
 */
STATIC inline bool Nav_HorizAidValid (uint32_t usNow) {

    if (!s_Nav.haveFix) {
        return false;
    }
    /* Unsigned subtraction, same wrap argument as Gps_HasFix(). */
    if ((usNow - s_Nav.usLastFixTime) > (CFG_NAV_HORIZ_AID_MAX_AGE_MS * 1000U)) {
        return false;
    }

    if (!s_Nav.haveVelRate) {
        return false;
    }

    /* Nothing to correct below this and only noise to inject - see
     * CFG_NAV_TRANS_ACCEL_MIN_SPEED_MPS. Gated on the FILTER's velocity, which
     * is the same speed smoothed, so the threshold does not chatter on receiver
     * noise the way the raw fix would. Compared squared: this runs at the
     * 400 Hz IMU rate and the root is not needed to order two non-negative
     * numbers. */
    float speedSq = (s_Nav.horizFilter.vel[0] * s_Nav.horizFilter.vel[0]) +
                    (s_Nav.horizFilter.vel[1] * s_Nav.horizFilter.vel[1]);
    if (speedSq < (CFG_NAV_TRANS_ACCEL_MIN_SPEED_MPS * CFG_NAV_TRANS_ACCEL_MIN_SPEED_MPS)) {
        return false;
    }

    /* Glitch guard: an over-range reading disables the correction for this
     * iteration rather than being clipped into range. */
    return (fabsf (s_Nav.transAccelNorth) <= CFG_NAV_TRANS_ACCEL_MAX_MPS2) &&
           (fabsf (s_Nav.transAccelEast) <= CFG_NAV_TRANS_ACCEL_MAX_MPS2);
}


/*
 * Horizontal acceleration in NED, m/s^2, north then east.
 *
 * The vertical partner above needs only the bottom row of the body->NED
 * matrix; these two need the top two, which is the whole of what
 * Firmware/CLAUDE.md means by the horizontal channels wanting the full
 * rotation.
 *
 * Same sign trap, and it is worth restating because it looks wrong. The
 * project's accel convention is g - a, so the rotated reading is
 * g_ned - a_ned; gravity has NO horizontal component, so the horizontal
 * acceleration is simply the NEGATED rotated specific force. Level and still
 * reads (0, 0, +9.81), which rotates to a horizontal pair of zeros either way -
 * so the sign cannot be checked at rest. Check it in a translation instead:
 * accelerating north reads -1 on body x when level, which must come out as
 * +1 north.
 */
STATIC void Nav_HorizontalAccelNed (Vec4f const* pQuat, Vec3f const* pAccel, float* pOutNed) {

    float w = pQuat->q1;
    float x = pQuat->q2;
    float y = pQuat->q3;
    float z = pQuat->q4;

    float fNorth = ((1.0F - (2.0F * ((y * y) + (z * z)))) * pAccel->x) +
                   ((2.0F * ((x * y) - (w * z))) * pAccel->y) +
                   ((2.0F * ((x * z) + (w * y))) * pAccel->z);
    float fEast = ((2.0F * ((x * y) + (w * z))) * pAccel->x) +
                  ((1.0F - (2.0F * ((x * x) + (z * z)))) * pAccel->y) +
                  ((2.0F * ((y * z) - (w * x))) * pAccel->z);

    pOutNed[0] = -fNorth;
    pOutNed[1] = -fEast;
}

/*
 * Remove the vehicle's own translational acceleration from the specific force,
 * leaving what the accelerometer would read if it were only feeling gravity.
 *
 * THE FAULT THIS EXISTS FOR (KnownIssues 1.20). An accelerometer cannot
 * separate gravity from acceleration. For a rotorcraft the specific force
 * points along the thrust axis whatever the attitude, so a translating vehicle
 * presents an apparent vertical rotated away from the real one - at very nearly
 * 1 g, which is why the magnitude band in filter.c cannot see it. Both this
 * filter and Betaflight's Mahony converge on that apparent vertical in trim, at
 * any gain, so the only fix is to stop presenting it.
 *
 * WHY THE SIGN LOOKS BACKWARDS. This project's accel convention is g - a (level
 * and still reads (0, 0, +9.81) in FRD - see Nav_VerticalAccelUp), so the
 * measured vector is R^T*(g_ned - a_ned) and recovering R^T*g_ned means ADDING
 * R^T*a_ned, not subtracting it. Check it against the level case: accelerating
 * north at 1 m/s^2 reads (-1, 0, 9.81), and +1 on x gives back (0, 0, 9.81).
 *
 * HORIZONTAL ONLY. a_ned's down component is left at zero. The vertical channel
 * knows its own acceleration, but it knows it from this same accelerometer
 * (AltitudeFilter_Predict is fed by Nav_VerticalAccelUp), so feeding it back in
 * here would close a loop between the estimate and its own input. GPS is the
 * only term in this correction that is independent of the IMU, and that
 * independence is the entire reason it can move the fixed point.
 */
STATIC inline void Nav_RemoveTranslation (Vec4f const* pQuat, Vec3f* pAccel, float aNorth, float aEast) {

    float w = pQuat->q1;
    float x = pQuat->q2;
    float y = pQuat->q3;
    float z = pQuat->q4;

    /* R is body->NED, so this is R^T applied to (aNorth, aEast, 0): the first
     * two COLUMNS of R, read as rows. Nav_VerticalAccelUp uses R's bottom row
     * out of the same matrix. */
    pAccel->x += ((1.0F - (2.0F * ((y * y) + (z * z)))) * aNorth) +
                 ((2.0F * ((x * y) + (w * z))) * aEast);
    pAccel->y += ((2.0F * ((x * y) - (w * z))) * aNorth) +
                 ((1.0F - (2.0F * ((x * x) + (z * z)))) * aEast);
    pAccel->z += ((2.0F * ((x * z) + (w * y))) * aNorth) +
                 ((2.0F * ((y * z) - (w * x))) * aEast);
}

/*
 * Angle between the measured specific force and where the estimate says down
 * is, in degrees. Zero when the two agree.
 *
 * This is the estimator's own innovation, and on hardware it is the ONLY
 * cross-check on the attitude that exists. KnownIssues 1.16 is the standing
 * version of that problem: the SIL's est_* gates work only because they have
 * FDM truth to compare against, and a real vehicle has no truth signal, so an
 * estimator that is confidently wrong is undetectable in flight. This does not
 * make it detectable either - it cannot separate "the accel is lying" from
 * "the estimate is wrong" - but it is the one number that goes non-zero when
 * either of them happens.
 *
 * It is DIRECTIONAL, which MadgwickFilter_AccelUsable's magnitude band is not,
 * and that is the whole point: 1.20's error is a rotation of the apparent
 * vertical at very nearly 1 g, so it passes the band untouched while showing
 * up here in full.
 *
 * Nav_VerticalAccelUp projects the accel onto the estimated down axis and
 * subtracts g, so adding g back recovers the projection itself. That axis is a
 * unit vector, so the projection over the accel magnitude is the cosine.
 */
STATIC float Nav_AccelResidualDeg (float accelUp, Vec3f const* pAccel) {
    float mag = sqrtf ((pAccel->x * pAccel->x) + (pAccel->y * pAccel->y) +
                       (pAccel->z * pAccel->z));

    /* Reported as 180, never as 0: a specific force too small to have a
     * direction carries no gravity information, and 0 would read as "the accel
     * agrees with the estimate" - the one thing it cannot mean here. */
    if (mag < NAV_ACCEL_MIN_MPS2) {
        return 180.0F;
    }

    float cosAngle = (accelUp + NAV_GRAVITY_MPS2) / mag;
    if (cosAngle > 1.0F) {
        cosAngle = 1.0F;
    }
    if (cosAngle < -1.0F) {
        cosAngle = -1.0F;
    }

    /*
     * Polynomial arccos rather than libm's acosf, because this runs once per
     * IMU sample at 400 Hz on a 64 MHz core (platform.c:133 - SYSCLK is HSI and
     * the configured PLL is never selected). Betaflight makes the same trade
     * for the same reason and this is its acos_approx (common/maths.c): max
     * error 3.9e-3 deg, one sqrtf and three multiply-adds.
     *
     * That is a cost argument, not a measured one, and it cannot be measured in
     * the SIL: the same binary there varies by 4.5x run to run under host load
     * (KnownIssues 1.17). Bench it if it ever needs to be a number.
     */
    float xa     = fabsf (cosAngle);
    float approx = sqrtf (1.0F - xa) *
                   (1.5707288F + xa * (-0.2121144F + xa * (0.0742610F + (-0.0187293F * xa))));
    return RAD2DEG (cosAngle < 0.0F ? (3.14159265F - approx) : approx);
}

/*
 * Latest-value cached read of the baro topic, the mag pattern - with one
 * addition that matters: the correction is skipped entirely on iterations where
 * no new sample arrived. Nav runs at the 400 Hz IMU rate against a ~50 Hz baro,
 * so re-applying a value already used would land the same correction eight
 * times instead of once, multiplying the effective gain by eight and turning a
 * 1 s time constant into an oscillation. Re-using a measurement does not
 * average it; it just amplifies it.
 */
STATIC void Nav_UpdateBaro (uint32_t usNow) {

    umsg_sensors_baro_t baro;
    if (!umsg_sensors_baro_receive (s_Nav.baro_sub, &baro, 0)) {
        return;
    }

    if (!s_Nav.haveDatum) {
        s_Nav.datumSumPa += baro.pressure;
        s_Nav.datumSumTempC += baro.temperature;
        s_Nav.datumSamples++;
        if (s_Nav.datumSamples < NAV_BARO_DATUM_SAMPLES) {
            return;
        }
        s_Nav.refPressurePa  = s_Nav.datumSumPa / (float)s_Nav.datumSamples;
        s_Nav.refTempC       = s_Nav.datumSumTempC / (float)s_Nav.datumSamples;
        s_Nav.haveDatum      = true;
        s_Nav.usLastBaroTime = usNow;
        LOG_INFO ("NAV baro datum %u Pa at %d C", (unsigned)s_Nav.refPressurePa,
                  (int)s_Nav.refTempC);
        return;
    }

    /* Unsigned subtraction, deliberately - same wrap argument as Gps_HasFix(). */
    float dtBaro         = (float)(usNow - s_Nav.usLastBaroTime) / 1000000.0F;
    s_Nav.usLastBaroTime = usNow;

    /* The DATUM temperature, not the current sample's: it scales height above
     * the datum, so it has to be the temperature of the air column's base. */
    AltitudeFilter_Correct (&s_Nav.altFilter, Baro_PressureToAltitude (baro.pressure, s_Nav.refPressurePa, s_Nav.refTempC), dtBaro);
}

/*
 * Latest-value cached read of the GPS topic. Position and velocity come
 * straight from the fix - no accel aiding - so they step at the receiver's
 * ~10 Hz and hold between fixes, while the vertical channel next door runs at
 * 400 Hz. That asymmetry is deliberate and worth knowing about: a position-hold
 * loop will want the smooth version, and building it is a separate job from
 * wiring the fix in.
 */
STATIC void Nav_UpdateGps (uint32_t usNow) {

    umsg_sensors_gps_t gps;
    if (!umsg_sensors_gps_receive (s_Nav.gps_sub, &gps, 0)) {
        return;
    }

    /*
     * Gps_Task republishes with fix_type 0 the moment the receiver loses lock
     * OR goes silent - it owns GPS_FIX_TIMEOUT_US - so validity here can simply
     * follow the message, and nav needs no timer of its own. A receiver that
     * never appears at all publishes nothing and haveFix stays false.
     *
     * The origin is NOT cleared on a lost fix. Re-deriving it from the first
     * fix after a dropout would silently move the NED frame under everything
     * holding a position expressed in it.
     */
    if (gps.fix_type < NAV_GPS_MIN_FIX_TYPE) {
        s_Nav.haveFix = false;
        /* Not just "stop updating": the next fix after a dropout would
         * otherwise be differenced against a velocity from before it, over a
         * dt spanning the whole outage. */
        s_Nav.haveVelRate = false;
        return;
    }

    if (!s_Nav.haveOrigin) {
        s_Nav.originLatDeg = gps.lat;
        s_Nav.originLonDeg = gps.lon;
        s_Nav.originCosLat = cosf (DEG2RAD ((float)gps.lat));
        s_Nav.haveOrigin   = true;
        LOG_INFO ("NAV origin set from first fix");
    }

    /* The doubles exist so the ABSOLUTE coordinate keeps its precision; the
     * difference from the origin is small enough that float carries it. */
    float dLatDeg = (float)(gps.lat - s_Nav.originLatDeg);
    float dLonDeg = (float)(gps.lon - s_Nav.originLonDeg);

    s_Nav.gpsPosNed[0] = DEG2RAD (dLatDeg) * NAV_EARTH_RADIUS_M;
    s_Nav.gpsPosNed[1] = DEG2RAD (dLonDeg) * NAV_EARTH_RADIUS_M * s_Nav.originCosLat;

    /* Course is degrees true, clockwise from north, so it resolves the ground
     * speed onto N/E directly. */
    float courseRad    = DEG2RAD (gps.course);
    s_Nav.gpsVelNed[0] = gps.speed * cosf (courseRad);
    s_Nav.gpsVelNed[1] = gps.speed * sinf (courseRad);

    /* Correct on the iterations a fix actually arrived, never on the ones in
     * between - the same argument as Nav_UpdateBaro's. Nav runs at 400 Hz
     * against a ~10 Hz receiver, so re-applying one fix would land the same
     * correction forty times and multiply the effective gain by forty.
     *
     * dt is the interval between FIXES for exactly that reason, not the nav
     * period. */
    if (s_Nav.haveFix) {
        float dtFix = (float)(usNow - s_Nav.usLastFixTime) / 1000000.0F;
        if (dtFix > 0.0F) {
            HorizontalFilter_Correct (&s_Nav.horizFilter, s_Nav.gpsPosNed, s_Nav.gpsVelNed, dtFix);
        }
    } else {
        /* First fix after boot or after a dropout: adopt it rather than
         * correcting towards it, so the estimate does not spend a time constant
         * walking from wherever it drifted to. */
        s_Nav.horizFilter.pos[0] = s_Nav.gpsPosNed[0];
        s_Nav.horizFilter.pos[1] = s_Nav.gpsPosNed[1];
        s_Nav.horizFilter.vel[0] = s_Nav.gpsVelNed[0];
        s_Nav.horizFilter.vel[1] = s_Nav.gpsVelNed[1];
    }

    /*
     * Differentiate only on the sentences that actually carried a velocity.
     *
     * Only RMC does; GGA carries a position and the driver leaves speed and
     * course at their previous values (drivers/gps/gps.c), so this function
     * runs at the ~10 Hz publish rate while the velocity underneath it moves at
     * 5 Hz. Differencing regardless would alternate a zero against a doubled
     * step - a 5 Hz ripple the size of the signal - and the dt would be the
     * publish interval rather than the interval the change happened over.
     * Comparing the values, not the timestamps, is also what makes this robust
     * to a receiver that repeats a sentence or drops one.
     */
    bool velMoved = (s_Nav.gpsVelNed[0] != s_Nav.lastVelNorth) ||
                    (s_Nav.gpsVelNed[1] != s_Nav.lastVelEast);
    if (s_Nav.haveVelRate && velMoved) {
        float dtVel = (float)(usNow - s_Nav.usLastVelTime) / 1000000.0F;
        if (dtVel > 0.0F) {
            s_Nav.transAccelNorth = LowPassFilter_Update (
            &s_Nav.transAccelFiltN, (s_Nav.gpsVelNed[0] - s_Nav.lastVelNorth) / dtVel, dtVel);
            s_Nav.transAccelEast = LowPassFilter_Update (
            &s_Nav.transAccelFiltE, (s_Nav.gpsVelNed[1] - s_Nav.lastVelEast) / dtVel, dtVel);
        }
    }
    if (velMoved || !s_Nav.haveVelRate) {
        s_Nav.lastVelNorth  = s_Nav.gpsVelNed[0];
        s_Nav.lastVelEast   = s_Nav.gpsVelNed[1];
        s_Nav.usLastVelTime = usNow;
        s_Nav.haveVelRate   = true;
    }

    s_Nav.usLastFixTime = usNow;
    s_Nav.haveFix       = true;

}

eSTATUS_t Nav_Init(void) {
    s_Nav.imu_sub          = umsg_sensors_imu_subscribe(1, 4);
    s_Nav.mag_sub          = umsg_sensors_mag_subscribe(1, 4);
    /* Depth 1: latest-value, and - because a length-1 queue overwrites - a
     * receive() that returns true is exactly "a new sample arrived", which is
     * what gates the baro correction and the GPS update below. */
    s_Nav.baro_sub         = umsg_sensors_baro_subscribe(1, 1);
    s_Nav.gps_sub          = umsg_sensors_gps_subscribe(1, 1);
    s_Nav.usLastUpdateTime = GetMicroseconds();
    s_Nav.haveFirstSample  = false;

    s_Nav.filter.cfg.gyroMeasureDriftDegs = CFG_GYRO_MEASURE_DRIFT_DEGS;
    s_Nav.filter.cfg.gyroMeasureErrorDegs = CFG_GYRO_MEASURE_ERROR_DEGS;

    s_Nav.horizFilter.cfg.kPos    = CFG_NAV_HORIZ_K_POS;
    s_Nav.horizFilter.cfg.kVel    = CFG_NAV_HORIZ_K_VEL;
    s_Nav.horizFilter.cfg.kBias   = CFG_NAV_HORIZ_K_BIAS;
    s_Nav.horizFilter.cfg.maxBias = CFG_NAV_HORIZ_MAX_BIAS_MPS2;
    if (STATUS_FAIL (HorizontalFilter_Init (&s_Nav.horizFilter))) {
        return eSTATUS_FAILURE;
    }

    s_Nav.transAccelFiltN.cfg.cutoffHz = CFG_NAV_TRANS_ACCEL_CUTOFF_HZ;
    s_Nav.transAccelFiltE.cfg.cutoffHz = CFG_NAV_TRANS_ACCEL_CUTOFF_HZ;
    if (STATUS_FAIL (LowPassFilter_Init (&s_Nav.transAccelFiltN)) ||
        STATUS_FAIL (LowPassFilter_Init (&s_Nav.transAccelFiltE))) {
        return eSTATUS_FAILURE;
    }

    s_Nav.altFilter.cfg.kAlt    = CFG_ALT_FILTER_K_ALT;
    s_Nav.altFilter.cfg.kVel    = CFG_ALT_FILTER_K_VEL;
    s_Nav.altFilter.cfg.kBias   = CFG_ALT_FILTER_K_BIAS;
    s_Nav.altFilter.cfg.maxBias = CFG_ALT_FILTER_MAX_BIAS_MPS2;

    eSTATUS_t status = AltitudeFilter_Init(&s_Nav.altFilter);
    if (STATUS_FAIL(status)) {
        return status;
    }
    return MadgwickFilter_Init(&s_Nav.filter);
}

eSTATUS_t Nav_Update(void) {
    umsg_sensors_imu_t imu_msg;
    if (!umsg_sensors_imu_receive(s_Nav.imu_sub, &imu_msg, portMAX_DELAY)) {
        return eSTATUS_FAILURE;
    }

    uint32_t usNow = GetMicroseconds();
    float dt = (float)(usNow - s_Nav.usLastUpdateTime) / 1000000.0f;
    s_Nav.usLastUpdateTime = usNow;

    /* Nav_Init() timestamps at boot but the first IMU sample only arrives once
     * the link is up - measured at 4.4 s in the SIL. Integrating that as one dt
     * throws the estimate ~14 deg past the true attitude and costs several
     * seconds of beta-limited slewing to recover. Seed the timestamp instead. */
    if (!s_Nav.haveFirstSample) {
        s_Nav.haveFirstSample = true;
        return eSTATUS_SUCCESS;
    }

    Vec3f accel;
    accel.x = imu_msg.accel[0];
    accel.y = imu_msg.accel[1];
    accel.z = imu_msg.accel[2];

    Vec3f gyro;
    gyro.x = imu_msg.gyro[0];
    gyro.y = imu_msg.gyro[1];
    gyro.z = imu_msg.gyro[2];

    umsg_sensors_mag_t mag_msg;
    Vec3f mag;
    Vec3f* pMag = NULL;
    if (umsg_sensors_mag_receive(s_Nav.mag_sub, &mag_msg, 0)) {
        mag.x = mag_msg.field[0];
        mag.y = mag_msg.field[1];
        mag.z = mag_msg.field[2];
        pMag  = &mag;
    }

    /*
     * Corrected for the filter, RAW for the altitude channel below. They want
     * different signals: the attitude filter wants gravity, the vertical
     * channel wants the true specific force along down, and taking the
     * horizontal translation out of the second one would be a no-op at best
     * (its NED down component is zero by construction) and injected GPS noise
     * at worst.
     */
    /*
     * One rotation, two consumers. accelNed is what the IMU says the vehicle is
     * doing horizontally, read through the attitude the estimate held when this
     * sample arrived; the horizontal filter integrates it, and subtracting that
     * filter's bias gives the best available estimate of the REAL acceleration -
     * the part GPS has agreed with. That is what the attitude correction wants.
     */
    float accelNed[HORIZ_AXES];
    Nav_HorizontalAccelNed (&s_Nav.filter.qEst, &accel, accelNed);

    Vec3f accelFilt = accel;
    if (Nav_HorizAidValid (usNow)) {
        Nav_RemoveTranslation ( &s_Nav.filter.qEst, 
                                &accelFilt,
                                s_Nav.transAccelNorth,
                                s_Nav.transAccelEast);
        s_Nav.filter.accelTrust = 1.0F;
    } else {
        /* No independent velocity, so the reading may be a false vertical and
         * nothing on board can tell. Slow the rate it is allowed to drag the
         * estimate at - see CFG_NAV_ACCEL_TRUST_UNAIDED for why this is a
         * compromise rather than a fix. */
        s_Nav.filter.accelTrust = CFG_NAV_ACCEL_TRUST_UNAIDED;
    }

    Vec3f euler;
    eSTATUS_t status = MadgwickFilter_Update(&s_Nav.filter, &accelFilt, &gyro, pMag, dt, &euler);
    if (STATUS_FAIL(status)) {
        return status;
    }

    /*
     * Vertical channel: predict from the IMU, correct from the baro.
     *
     * Prediction is gated on the datum existing. Before it does there is nothing
     * for the accel to be integrated RELATIVE to, so integrating anyway would
     * only accumulate bias into a number that is about to be defined as zero.
     * Ordering is still predict-then-correct: on the iteration the datum
     * completes, haveDatum is false here and true from the next one on.
     */
    float accelUp = Nav_VerticalAccelUp (&s_Nav.filter.qEst, &accel);
    if (s_Nav.haveDatum) {
        AltitudeFilter_Predict(&s_Nav.altFilter, accelUp, dt);
    }
    Nav_UpdateBaro(usNow);

    /* Predict-then-correct, and unconditional unlike the vertical channel's:
     * the horizontal states are all RELATIVE to an origin, so integrating
     * before there is a fix costs only that the origin has not been named yet.
     * The altitude channel gates its predict because its datum defines zero. */
    HorizontalFilter_Predict (&s_Nav.horizFilter, accelNed, dt);
    Nav_UpdateGps(usNow);

    umsg_nav_state_t msg;
    msg.quat[0]    = s_Nav.filter.qEst.q1;
    msg.quat[1]    = s_Nav.filter.qEst.q2;
    msg.quat[2]    = s_Nav.filter.qEst.q3;
    msg.quat[3]    = s_Nav.filter.qEst.q4;
    msg.euler[0]   = euler.x;
    msg.euler[1]   = euler.y;
    msg.euler[2]   = euler.z;
    msg.gyro[0]    = imu_msg.gyro[0];
    msg.gyro[1]    = imu_msg.gyro[1];
    msg.gyro[2]    = imu_msg.gyro[2];
    /* pos_ned/vel_ned are NED, down positive; the altitude filter works up
     * positive because that is what an altitude reads like. Hence the negation
     * on the third element, and only there. */
    msg.pos_ned[0] = s_Nav.horizFilter.pos[0];
    msg.pos_ned[1] = s_Nav.horizFilter.pos[1];
    msg.pos_ned[2] = -s_Nav.altFilter.alt;
    msg.vel_ned[0] = s_Nav.horizFilter.vel[0];
    msg.vel_ned[1] = s_Nav.horizFilter.vel[1];
    msg.vel_ned[2] = -s_Nav.altFilter.vz;
    msg.alt        = s_Nav.altFilter.alt;
    /* On the CORRECTED accel, i.e. on what the filter was actually given.
     * That makes it the filter's own innovation: near zero says the input was
     * consistent with the state, whatever it took to get there. Reporting the
     * raw disagreement instead would read large through every translation even
     * when the correction above is working perfectly, which is the opposite of
     * what a health monitor should do. */
    msg.accel_residual_deg =
    Nav_AccelResidualDeg (Nav_VerticalAccelUp (&s_Nav.filter.qEst, &accelFilt), &accelFilt);

    /*
     * The three flags do not partition the vectors the way their names suggest,
     * because the sensors do not either. Baro owns the vertical, GPS owns the
     * horizontal, and each can be valid while the other is not:
     *
     *   NAV_VALID_BARO_ALT -> alt, pos_ned[2], vel_ned[2]
     *   NAV_VALID_POSITION -> pos_ned[0..1]
     *   NAV_VALID_VELOCITY -> vel_ned[0..1]
     */
    uint8_t valid = NAV_VALID_ATTITUDE;
    if (s_Nav.haveDatum) {
        valid |= NAV_VALID_BARO_ALT;
    }
    if (s_Nav.haveFix) {
        valid |= NAV_VALID_POSITION | NAV_VALID_VELOCITY;
    }
    msg.valid = valid;
    umsg_nav_state_publish(&msg);
    return eSTATUS_SUCCESS;
}

void Nav_Task(void* args) {
    (void)args;
    Nav_Init();
    while (1) {
        Nav_Update();
    }
}
