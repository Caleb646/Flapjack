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
    for (uint32_t i = 0; i < motorCount; ++i) {
        float mixedThrottle = pMix[i].throttle * pidData[AXIS_IDX_THROTTLE];
        mixedThrottle += pMix[i].pitch * ABS_F32 (pidData[AXIS_IDX_PITCH]);
        mixedThrottle += pMix[i].roll * pidData[AXIS_IDX_ROLL];
        mixedThrottle += pMix[i].yaw * pidData[AXIS_IDX_YAW];
        motorOutputs[i] = clipf32 (mixedThrottle, pMixer->motorsMinThrottle, pMixer->motorsMaxThrottle);
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

    /* Accumulate the signed mix contributions in float, then convert to a pulse
     * width centred on SERVO_CENTER_US_DC and clamp to the servo's legal travel.
     * Summing straight into the uint16_t output wraps on any negative
     * contribution and never clamps, so the driver saw out-of-range widths. */
    for (uint32_t i = 0; i < BRD_SERVO_COUNT; ++i) {
        float us = (float)SERVO_CENTER_US_DC +
                   (mixedInputs[i] * (float)(SERVO_RIGHT_US_DC - SERVO_CENTER_US_DC));
        servoOutputs[i] = (uint16_t)clipf32 (us, (float)SERVO_LEFT_US_DC, (float)SERVO_RIGHT_US_DC);
    }
}

eSTATUS_t Mixer_Init_ (eMIXER_PROFILE_ID_t profileId, Mixer_t* pOutMixer) {

    pOutMixer->pCurrentProfile   = &pOutMixer->profiles[profileId];
    pOutMixer->motorsMinThrottle = 0.0F;
    pOutMixer->motorsMaxThrottle = 1.0F;
    return eSTATUS_SUCCESS;
}


eSTATUS_t Mixer_Init (void) {
    // TODO: set profileId based on cfg file
    return Mixer_Init_ (eMIXER_PROFILE_TILT_ROTOR, &g_Mixer);
}