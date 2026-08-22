#include <stdint.h>

#include "target.h"

#include "core/core.h"

#include "tasks/control/mixer.h"
#include "devices/motors.h"
#include "devices/servos.h"

// clang-format off
FJ_DEFINE_SHARED (MotorMix_t, g_TestMotorMix[]) = {
    { .throttle = 1.0F, .roll = 0.0F, .pitch = 0.0F, .yaw = 0.0F }
};

FJ_DEFINE_SHARED (ServoMix_t, g_TestServoMix[]) = {
    { eSERVO_LEFT_MOTOR_ID, eSERVO_MIX_INPUT_PID_ROLL,     1.0F },
    { eSERVO_LEFT_MOTOR_ID, eSERVO_MIX_INPUT_PID_PITCH,    1.0F },
    { eSERVO_LEFT_MOTOR_ID, eSERVO_MIX_INPUT_PID_YAW,      1.0F },
    { eSERVO_LEFT_MOTOR_ID, eSERVO_MIX_INPUT_PID_THROTTLE, 1.0F }
};
// clang-format on

/*
 * Twin tilt-rotor ("bicopter") mixing. Body frame is FRD: +roll = right side
 * down, +pitch = nose up, +yaw = nose right. The two rotors sit on the lateral
 * axis, left at -y and right at +y, each on a servo that tilts it fore/aft
 * (mixer output +1.0 = rotor tilted fully FORWARD, see drivers/servo/sim.c).
 *
 * That geometry gives three independent authorities:
 *   roll  - differential THRUST   (more thrust on the left rolls right)
 *   yaw   - differential TILT     (left rotor forward yaws the nose right)
 *   pitch - collective TILT       (rotors sit above the CG, so tilting them
 *                                  forward pitches the nose DOWN; hence -1.0)
 *
 * Throttle drives both rotors together and must NOT reach the tilt servos.
 */
FJ_DEFINE_SHARED (MotorMix_t, g_TiltMotorMix[]) = {
    { .throttle = 1.0F, .roll = 1.0F,  .pitch = 0.0F, .yaw = 0.0F }, // Left
    { .throttle = 1.0F, .roll = -1.0F, .pitch = 0.0F, .yaw = 0.0F }, // Right
};

FJ_DEFINE_SHARED (ServoMix_t, g_TiltServoMix[]) = {
    { eSERVO_LEFT_MOTOR_ID,  eSERVO_MIX_INPUT_PID_PITCH, -1.0F },
    { eSERVO_LEFT_MOTOR_ID,  eSERVO_MIX_INPUT_PID_YAW,    1.0F },

    { eSERVO_RIGHT_MOTOR_ID, eSERVO_MIX_INPUT_PID_PITCH, -1.0F },
    { eSERVO_RIGHT_MOTOR_ID, eSERVO_MIX_INPUT_PID_YAW,   -1.0F },
};

FJ_DEFINE_SHARED (MotorMix_t, g_AirplaneMotorMix[]) = {
    { .throttle = 1.0F, .roll = 0.0F, .pitch = 0.0F, .yaw = 0.0F }, // Left
    { .throttle = 1.0F, .roll = 0.0F, .pitch = 0.0F, .yaw = 0.0F }, // Right
};

// clang-format off
FJ_DEFINE_SHARED (ServoMix_t, g_AirplaneServoMix[]) = {
    { eSERVO_LEFT_AILERON_ID,  eSERVO_MIX_INPUT_PID_ROLL,  1.0F },

    { eSERVO_RIGHT_AILERON_ID, eSERVO_MIX_INPUT_PID_ROLL,  1.0F },

    { eSERVO_ELEVATOR_ID,      eSERVO_MIX_INPUT_PID_PITCH, 1.0F },

    { eSERVO_RUDDER_ID,        eSERVO_MIX_INPUT_PID_YAW,   1.0F }
};
// clang-format on

FJ_DEFINE_SHARED (Mixer_t, g_Mixer) = {
    .profiles = {
        [eMIXER_PROFILE_TEST_SETUP] = {
            .motorCount    = 1U,
            .servoCount    = 1U,
            .pMotorMix     = g_TestMotorMix,
            .pServoMix     = g_TestServoMix,
            .servoMixCount = ARRAY_SIZE (g_TestServoMix),
        },
        [eMIXER_PROFILE_TILT_ROTOR] = {
            .motorCount    = 2U,
            .servoCount    = 2U,
            .pMotorMix     = g_TiltMotorMix,
            .pServoMix     = g_TiltServoMix,
            .servoMixCount = ARRAY_SIZE (g_TiltServoMix),
        },
        [eMIXER_PROFILE_AIRPLANE] = {
            .motorCount    = 2U,
            .servoCount    = 8U,
            .pMotorMix     = g_AirplaneMotorMix,
            .pServoMix     = g_AirplaneServoMix,
            .servoMixCount = ARRAY_SIZE (g_AirplaneServoMix),
        },
    }
};

void Mixer_MixMotors (Mixer_t* pMixer, float const pidData[AXIS_IDX_COUNT], float motorOutputs[BRD_MOTOR_COUNT]) {

    MixerProfile_t const* pProfile = pMixer->pCurrentProfile;
    MotorMix_t const* pMix         = pProfile->pMotorMix;
    /* A profile may declare more motors than this board fits (the tilt and
     * airplane profiles want 2; nucleo-h747zi has 1), so bound by the board. */
    uint32_t motorCount = pProfile->motorCount;
    if (motorCount > BRD_MOTOR_COUNT) {
        motorCount = BRD_MOTOR_COUNT;
    }
    /*
     * Split the mix into the part that must survive and the part that may be
     * spent. Roll and yaw set the DIFFERENCE between motors; throttle and the
     * pitch lift term move them together.
     *
     * Clipping each motor independently - as this did - spends the difference
     * first, and on a bicopter differential thrust is the only roll effector:
     * near full throttle the upper motor pins while the lower one still moves,
     * so the roll response halves and goes asymmetric exactly where it is
     * needed most.
     *
     * ABS on pitch is deliberate and belongs on the collective side. Tilting the
     * rotors costs vertical thrust whichever way they go, so that term is lift
     * compensation, not a pitch effector (see the mixing note in cfgs/cfg.h).
     */
    float collective[BRD_MOTOR_COUNT];
    float differential[BRD_MOTOR_COUNT];
    float diffMin = 0.0F;
    float diffMax = 0.0F;

    for (uint32_t i = 0; i < motorCount; ++i) {
        collective[i] = (pMix[i].throttle * pidData[AXIS_IDX_THROTTLE]) +
                        (pMix[i].pitch * ABS_F32 (pidData[AXIS_IDX_PITCH]));
        differential[i] = (pMix[i].roll * pidData[AXIS_IDX_ROLL]) +
                          (pMix[i].yaw * pidData[AXIS_IDX_YAW]);

        if ((i == 0U) || (differential[i] < diffMin)) {
            diffMin = differential[i];
        }
        if ((i == 0U) || (differential[i] > diffMax)) {
            diffMax = differential[i];
        }
    }

    /* A differential wider than the whole throttle band cannot be delivered at
     * any offset, so scale it down and let every motor lose authority by the
     * same factor. That keeps the roll response symmetric about the stick
     * instead of letting whichever motor clips first set its shape. */
    float band  = pMixer->motorsMaxThrottle - pMixer->motorsMinThrottle;
    float range = diffMax - diffMin;
    float scale = 1.0F;
    if (range > band) {
        scale = band / range;
    }

    float mixed[BRD_MOTOR_COUNT];
    float mixedMin = 0.0F;
    float mixedMax = 0.0F;
    for (uint32_t i = 0; i < motorCount; ++i) {
        mixed[i] = collective[i] + (differential[i] * scale);

        if ((i == 0U) || (mixed[i] < mixedMin)) {
            mixedMin = mixed[i];
        }
        if ((i == 0U) || (mixed[i] > mixedMax)) {
            mixedMax = mixed[i];
        }
    }

    /* Slide the set into the band as a block. Shifting spends collective thrust,
     * which the throttle/altitude loop trims back within a few frames; clipping
     * would instead spend the differential, which nothing downstream recovers. */
    float offset = 0.0F;
    if (mixedMax > pMixer->motorsMaxThrottle) {
        offset = pMixer->motorsMaxThrottle - mixedMax;
    }
    if ((mixedMin + offset) < pMixer->motorsMinThrottle) {
        offset = pMixer->motorsMinThrottle - mixedMin;
    }

    for (uint32_t i = 0; i < motorCount; ++i) {
        motorOutputs[i] = clipf32 (mixed[i] + offset, pMixer->motorsMinThrottle, pMixer->motorsMaxThrottle);
    }
}

void Mixer_MixServos (Mixer_t* pMixer, float const pidData[AXIS_IDX_COUNT], uint16_t servoOutputs[BRD_SERVO_COUNT]) {

    MixerProfile_t const* pProfile = pMixer->pCurrentProfile;
    ServoMix_t const* pServoMix    = pProfile->pServoMix;
    // TODO: Some servos will need access to Rc channel data for flight mode switching

    float mixedInputs[BRD_SERVO_COUNT] = { 0.0F };
    for (uint32_t mixIdx = 0; mixIdx < pProfile->servoMixCount; ++mixIdx) {
        /* A profile may name servo IDs this board does not fit - the airplane
         * profile goes up to ID 6 while BRD_SERVO_COUNT is 2 - so drop those
         * entries instead of writing past mixedInputs. eSERVO_ID_NONE underflows
         * to a huge index and is caught by the same test. */
        uint32_t servoIdx = SERVO_ID_TO_IDX (pServoMix[mixIdx].targetServo);
        if (servoIdx >= BRD_SERVO_COUNT) {
            continue;
        }

        float mixedInput = 0.0F;
        switch (pServoMix[mixIdx].inputIndex) {
        case eSERVO_MIX_INPUT_PID_ROLL: mixedInput = pidData[AXIS_IDX_ROLL]; break;
        case eSERVO_MIX_INPUT_PID_PITCH: mixedInput = pidData[AXIS_IDX_PITCH]; break;
        case eSERVO_MIX_INPUT_PID_YAW: mixedInput = pidData[AXIS_IDX_YAW]; break;
        case eSERVO_MIX_INPUT_PID_THROTTLE: mixedInput = pidData[AXIS_IDX_THROTTLE]; break;
        default: break;
        }
        mixedInputs[servoIdx] += mixedInput * pServoMix[mixIdx].weight;
    }

    /*
     * Bound total deflection BEFORE the pulse-width conversion, so the limit is
     * expressed in tilt angle rather than in whatever travel the servo
     * calibration happens to allow. Pitch and yaw both drive the tilt servos at
     * weight 1.0, so the unbounded sum reaches +/-2 - rotors pinned horizontal
     * on any saturated rate loop.
     *
     * Scale the whole SET by one factor rather than clipping each servo, for the
     * reason Mixer_MixMotors gives for its differential: several servos carry
     * the same axes with different signs, so their values are a decomposition,
     * not independent numbers. Clipping them separately does not attenuate the
     * axes evenly - it hits whichever axis is asking for LESS far harder, and
     * pitch is already this airframe's weak axis. Measured in the SIL on the
     * tilt pair (left = -pitch + yaw, right = -pitch - yaw), flying full pitch
     * and 0.8 yaw together: clipping kept 81-87% of the pitch command against
     * 97-98% of the yaw, where one common factor keeps 94-96% of both.
     *
     * The gap grows with saturation depth - it is 5x here at 5% past the limit,
     * and a constructed pitch 0.2 / yaw 0.5 (110% past) loses 92% of the pitch
     * against 37% of the yaw. Real flight on the current tune only reaches the
     * shallow end, and rarely: 7 frames in a 30 s aggressive manoeuvre. This is
     * cheap insurance that scales with the tune, not a hot path.
     *
     * One common factor costs both axes the same fraction and leaves the
     * commanded direction in the (pitch, yaw) plane intact.
     *
     * There is no counterpart to the motors' second step, the common offset.
     * That band is asymmetric about the operating point, so sliding within it
     * spends collective thrust; this limit is symmetric about centre, so the
     * same offset would just be an uncommanded collective tilt - a pitch
     * command, not a desaturation.
     *
     * The factor is taken over every servo the profile drives. For the tilt
     * pair that is exactly right. It does couple servos that share no inputs -
     * an airplane profile's elevator saturating would trim the ailerons too -
     * which is a uniform authority loss rather than a distorted one, but worth
     * revisiting if that profile is ever wired up (Mixer_Init still hardcodes
     * the tilt-rotor).
     */
    float maxAbs = 0.0F;
    for (uint32_t i = 0; i < BRD_SERVO_COUNT; ++i) {
        float magnitude = ABS_F32 (mixedInputs[i]);
        if (magnitude > maxAbs) {
            maxAbs = magnitude;
        }
    }
    float scale = 1.0F;
    if (maxAbs > pMixer->servoTravelLimit) {
        scale = pMixer->servoTravelLimit / maxAbs;
    }

    /* Accumulate the signed mix contributions in float, then convert to a pulse
     * width centred on SERVO_CENTER_US_DC and clamp to the servo's legal travel.
     * Summing straight into the uint16_t output wraps on any negative
     * contribution and never clamps, so the driver saw out-of-range widths. */
    for (uint32_t i = 0; i < BRD_SERVO_COUNT; ++i) {
        /* The scale above already puts every servo inside the limit, so this
         * clip is NOT the desaturation - it is the NaN guard. A NaN mix survives
         * both the max scan (every comparison against it is false) and the
         * multiply, and clipf32 is what keeps it from reaching the pulse-width
         * cast; see the NaN note on clipf32 itself, and KnownIssues 1.14 and
         * 2.25. Removing it as redundant restores that bug. */
        float travel = clipf32 (mixedInputs[i] * scale, -pMixer->servoTravelLimit,
                                pMixer->servoTravelLimit);

        float us = (float)SERVO_CENTER_US_DC +
                   (travel * (float)(SERVO_RIGHT_US_DC - SERVO_CENTER_US_DC));
        servoOutputs[i] = (uint16_t)clipf32 (us, (float)SERVO_LEFT_US_DC, (float)SERVO_RIGHT_US_DC);
    }
}

eSTATUS_t Mixer_Init_ (eMIXER_PROFILE_ID_t profileId, Mixer_t* pOutMixer) {

    pOutMixer->pCurrentProfile   = &pOutMixer->profiles[profileId];
    pOutMixer->motorsMinThrottle = CFG_MIXER_IDLE_THROTTLE;
    pOutMixer->motorsMaxThrottle = CFG_MIXER_MAX_THROTTLE;
    pOutMixer->servoTravelLimit  = CFG_MIXER_SERVO_TRAVEL_LIMIT;
    return eSTATUS_SUCCESS;
}


eSTATUS_t Mixer_Init (void) {
    // TODO: set profileId based on cfg file
    return Mixer_Init_ (eMIXER_PROFILE_TILT_ROTOR, &g_Mixer);
}