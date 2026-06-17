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
    { eSERVO_LEFT_MOTOR_ID, eSERVO_MIX_INPUT_PID_ROLL },
    { eSERVO_LEFT_MOTOR_ID, eSERVO_MIX_INPUT_PID_PITCH },
    { eSERVO_LEFT_MOTOR_ID, eSERVO_MIX_INPUT_PID_YAW },
    { eSERVO_LEFT_MOTOR_ID, eSERVO_MIX_INPUT_PID_THROTTLE }
};
// clang-format on

FJ_DEFINE_SHARED (MotorMix_t, g_TiltMotorMix[]) = {
    { .throttle = 1.0F, .roll = 0.0F, .pitch = 0.0F, .yaw = 0.0F }, // Left
    { .throttle = 1.0F, .roll = 0.0F, .pitch = 0.0F, .yaw = 0.0F }, // Right
};

FJ_DEFINE_SHARED (ServoMix_t, g_TiltServoMix[]) = {
    { eSERVO_LEFT_MOTOR_ID, eSERVO_MIX_INPUT_PID_ROLL },
    { eSERVO_LEFT_MOTOR_ID, eSERVO_MIX_INPUT_PID_PITCH },
    { eSERVO_LEFT_MOTOR_ID, eSERVO_MIX_INPUT_PID_YAW },
    { eSERVO_LEFT_MOTOR_ID, eSERVO_MIX_INPUT_PID_THROTTLE },

    { eSERVO_RIGHT_MOTOR_ID, eSERVO_MIX_INPUT_PID_ROLL },
    { eSERVO_RIGHT_MOTOR_ID, eSERVO_MIX_INPUT_PID_PITCH },
    { eSERVO_RIGHT_MOTOR_ID, eSERVO_MIX_INPUT_PID_YAW },
    { eSERVO_RIGHT_MOTOR_ID, eSERVO_MIX_INPUT_PID_THROTTLE },
};

FJ_DEFINE_SHARED (MotorMix_t, g_AirplaneMotorMix[]) = {
    { .throttle = 1.0F, .roll = 0.0F, .pitch = 0.0F, .yaw = 0.0F }, // Left
    { .throttle = 1.0F, .roll = 0.0F, .pitch = 0.0F, .yaw = 0.0F }, // Right
};

// clang-format off
FJ_DEFINE_SHARED (ServoMix_t, g_AirplaneServoMix[]) = {
    { eSERVO_LEFT_AILERON_ID, eSERVO_MIX_INPUT_PID_ROLL },

    { eSERVO_RIGHT_AILERON_ID, eSERVO_MIX_INPUT_PID_ROLL },

    { eSERVO_ELEVATOR_ID, eSERVO_MIX_INPUT_PID_PITCH },

    { eSERVO_RUDDER_ID, eSERVO_MIX_INPUT_PID_YAW }
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
            .servoCount    = 3U,
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
    for (uint32_t i = 0; i < pProfile->motorCount; ++i) {
        float mixedThrottle = pMix[i].throttle * pidData[AXIS_IDX_THROTTLE];
        mixedThrottle += pMix[i].pitch * ABS_F32 (pidData[AXIS_IDX_PITCH]);
        mixedThrottle += pMix[i].roll * pidData[AXIS_IDX_ROLL];
        mixedThrottle += pMix[i].yaw * pidData[AXIS_IDX_YAW];
        motorOutputs[i] = clipf32 (mixedThrottle, pMixer->motorsMinThrottle, pMixer->motorsMaxThrottle);
    }
}

void Mixer_MixServos (Mixer_t* pMixer, float const pidData[AXIS_IDX_COUNT], uint16_t servoOutputs[BRD_SERVO_COUNT]) {

    MixerProfile_t const* pProfile          = pMixer->pCurrentProfile;
    ServoMix_t const* pServoMix             = pProfile->pServoMix;
    uint16_t inputs[eSERVO_MIX_INPUT_COUNT] = { 0 };
    // TODO: Should PID be between -1 and 1 or another range????
    // TODO: Some servos will need access to Rc channel data for flight mode switching
    inputs[eSERVO_MIX_INPUT_PID_ROLL] =
    (uint16_t)mapf32 (pidData[AXIS_IDX_ROLL], -1.0F, 1.0F, SERVO_LEFT_US_DC, SERVO_RIGHT_US_DC);

    for (uint32_t mixIdx = 0; mixIdx < pProfile->servoMixCount; ++mixIdx) {
        float mixedInput = 0.0F;
        switch (pServoMix[mixIdx].inputIndex) {
        case eSERVO_MIX_INPUT_PID_ROLL: mixedInput = pidData[AXIS_IDX_ROLL]; break;
        case eSERVO_MIX_INPUT_PID_PITCH: mixedInput = pidData[AXIS_IDX_PITCH]; break;
        case eSERVO_MIX_INPUT_PID_YAW: mixedInput = pidData[AXIS_IDX_YAW]; break;
        case eSERVO_MIX_INPUT_PID_THROTTLE: mixedInput = pidData[AXIS_IDX_THROTTLE]; break;
        default: break;
        }
        // TODO: Scale mixedInput to appropriate servo output range
        servoOutputs[SERVO_ID_TO_IDX (pServoMix[mixIdx].targetServo)] += (uint16_t)(mixedInput * 1000.0F);
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