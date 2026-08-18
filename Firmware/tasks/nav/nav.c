#include "tasks/nav/nav.h"
#include "common/filter.h"
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
 * A 2D fix is enough. Altitude comes from the baro, not from GPS, so the third
 * dimension the receiver would add is one this estimator does not consume.
 */
#define NAV_GPS_MIN_FIX_TYPE 2U

typedef struct {
    MadgwickFilter_t filter;
    AltitudeFilter_t altFilter;

    umsg_sub_handle_t imu_sub;
    umsg_sub_handle_t mag_sub;
    umsg_sub_handle_t baro_sub;
    umsg_sub_handle_t gps_sub;

    uint32_t usLastUpdateTime;
    bool haveFirstSample;

    /* Pressure datum, averaged over the first NAV_BARO_DATUM_SAMPLES samples. */
    float datumSumPa;
    uint32_t datumSamples;
    float refPressurePa;
    bool haveDatum;
    uint32_t usLastBaroTime;

    /* Flat-earth origin, captured once at the first usable fix. */
    double originLatDeg;
    double originLonDeg;
    float originCosLat;
    bool haveOrigin;
    bool haveFix;

    float posNorth;
    float posEast;
    float velNorth;
    float velEast;
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
STATIC float Nav_VerticalAccelUp (Vec4f const* pQuat, Vec3f const* pAccel) {

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
        s_Nav.datumSamples++;
        if (s_Nav.datumSamples < NAV_BARO_DATUM_SAMPLES) {
            return;
        }
        s_Nav.refPressurePa  = s_Nav.datumSumPa / (float)s_Nav.datumSamples;
        s_Nav.haveDatum      = true;
        s_Nav.usLastBaroTime = usNow;
        LOG_INFO ("NAV baro datum %u Pa", (unsigned)s_Nav.refPressurePa);
        return;
    }

    /* Unsigned subtraction, deliberately - same wrap argument as Gps_HasFix(). */
    float dtBaro         = (float)(usNow - s_Nav.usLastBaroTime) / 1000000.0F;
    s_Nav.usLastBaroTime = usNow;

    AltitudeFilter_Correct (&s_Nav.altFilter,
                            Baro_PressureToAltitude (baro.pressure, s_Nav.refPressurePa),
                            dtBaro);
}

/*
 * Latest-value cached read of the GPS topic. Position and velocity come
 * straight from the fix - no accel aiding - so they step at the receiver's
 * ~10 Hz and hold between fixes, while the vertical channel next door runs at
 * 400 Hz. That asymmetry is deliberate and worth knowing about: a position-hold
 * loop will want the smooth version, and building it is a separate job from
 * wiring the fix in.
 */
STATIC void Nav_UpdateGps (void) {

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

    s_Nav.posNorth = DEG2RAD (dLatDeg) * NAV_EARTH_RADIUS_M;
    s_Nav.posEast  = DEG2RAD (dLonDeg) * NAV_EARTH_RADIUS_M * s_Nav.originCosLat;

    /* Course is degrees true, clockwise from north, so it resolves the ground
     * speed onto N/E directly. */
    float courseRad = DEG2RAD (gps.course);
    s_Nav.velNorth  = gps.speed * cosf (courseRad);
    s_Nav.velEast   = gps.speed * sinf (courseRad);
    s_Nav.haveFix   = true;
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

    Vec3f euler;
    eSTATUS_t status = MadgwickFilter_Update(&s_Nav.filter, &accel, &gyro, pMag, dt, &euler);
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
    if (s_Nav.haveDatum) {
        AltitudeFilter_Predict(&s_Nav.altFilter,
                               Nav_VerticalAccelUp(&s_Nav.filter.qEst, &accel), dt);
    }
    Nav_UpdateBaro(usNow);
    Nav_UpdateGps();

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
    msg.pos_ned[0] = s_Nav.posNorth;
    msg.pos_ned[1] = s_Nav.posEast;
    msg.pos_ned[2] = -s_Nav.altFilter.alt;
    msg.vel_ned[0] = s_Nav.velNorth;
    msg.vel_ned[1] = s_Nav.velEast;
    msg.vel_ned[2] = -s_Nav.altFilter.vz;
    msg.alt        = s_Nav.altFilter.alt;

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
