#include <stdint.h>

#include "common.h"

#include "drivers/motor.h"

#include "aero/flight.h"
#include "aero/mixer.h"
#include "aero/pid.h"

#include "targets/target.h"


FJ_DEFINE_SHARED (MotorMix_t, e_TiltMotorMix[]) = {
    { .throttle = 1.0F, .roll = 0.0F, .pitch = 0.0F, .yaw = 0.0F }, // Left
    { .throttle = 1.0F, .roll = 0.0F, .pitch = 0.0F, .yaw = 0.0F }, // Right
};

FJ_DEFINE_SHARED (ServoMix_t, e_TiltServoMix[]) = {
    { eSERVO_LEFT_MOTOR_ID, eSERVO_MIX_INPUT_PID_ROLL },
    { eSERVO_LEFT_MOTOR_ID, eSERVO_MIX_INPUT_PID_PITCH },
    { eSERVO_LEFT_MOTOR_ID, eSERVO_MIX_INPUT_PID_YAW },
    { eSERVO_LEFT_MOTOR_ID, eSERVO_MIX_INPUT_PID_THROTTLE },

    { eSERVO_RIGHT_MOTOR_ID, eSERVO_MIX_INPUT_PID_ROLL },
    { eSERVO_RIGHT_MOTOR_ID, eSERVO_MIX_INPUT_PID_PITCH },
    { eSERVO_RIGHT_MOTOR_ID, eSERVO_MIX_INPUT_PID_YAW },
    { eSERVO_RIGHT_MOTOR_ID, eSERVO_MIX_INPUT_PID_THROTTLE },
};

// clang-format off
FJ_DEFINE_SHARED (ServoMix_t, e_AirplaneServoMix[]) = {
    { eSERVO_LEFT_AILERON_ID, eSERVO_MIX_INPUT_PID_ROLL },

    { eSERVO_RIGHT_AILERON_ID, eSERVO_MIX_INPUT_PID_ROLL },

    { eSERVO_ELEVATOR_ID, eSERVO_MIX_INPUT_PID_PITCH },

    { eSERVO_RUDDER_ID, eSERVO_MIX_INPUT_PID_YAW }
};
// clang-format on

FJ_DEFINE_SHARED (Mixer_t, e_Mixer) = {
    .profiles = {
        [eMIXER_PROFILE_TILT_ROTOR] = {
            .motorCount    = 2U,
            .servoCount    = 2U,
            .pMotorMix     = e_TiltMotorMix,
            .pServoMix     = e_TiltServoMix,
            .servoMixCount = ARRAY_SIZE (e_TiltServoMix),
        },
        [eMIXER_PROFILE_AIRPLANE] = {
            .motorCount    = 2U,
            .servoCount    = 4U,
            .pMotorMix     = e_TiltMotorMix,
            .pServoMix     = e_AirplaneServoMix,
            .servoMixCount = ARRAY_SIZE (e_AirplaneServoMix),
        },
    }
};

FJ_TESTABLE FJ_INLINE void
Mixer_MixMotors (Mixer_t* pMixer, float const pidAttitude[AXIS_IDX_COUNT], float targetThrottle, float motorOutputs[TARG_MAX_MOTORS]) {

    MixerProfile_t const* pProfile = &pMixer->profiles[pMixer->currentProfileId];
    MotorMix_t const* pMix         = pProfile->pMotorMix;

    for (uint32_t i = 0; i < pProfile->motorCount; ++i) {
        float mixedThrottle = pMix[i].throttle * targetThrottle;
        mixedThrottle += pMix[i].pitch * ABS_F32 (pidAttitude[AXIS_IDX_PITCH]);
        mixedThrottle += pMix[i].roll * pidAttitude[AXIS_IDX_ROLL];
        mixedThrottle += pMix[i].yaw * pidAttitude[AXIS_IDX_YAW];
        motorOutputs[i] = clipf32 (mixedThrottle, pMixer->motorsMinThrottle, pMixer->motorsMaxThrottle);
    }
}

FJ_TESTABLE FJ_INLINE void
Mixer_MixServos (Mixer_t* pMixer, float const pidAttitude[AXIS_IDX_COUNT], float targetThrottle, uint16_t servoOutputs[TARG_MAX_SERVOS]) {

    MixerProfile_t const* pProfile          = &pMixer->profiles[pMixer->currentProfileId];
    ServoMix_t const* pServoMix             = pProfile->pServoMix;
    uint16_t inputs[eSERVO_MIX_INPUT_COUNT] = { 0 };
    // TODO: Should PID be between -1 and 1 or another range????
    inputs[eSERVO_MIX_INPUT_PID_ROLL] =
    (uint16_t)mapf32 (pidAttitude[AXIS_IDX_ROLL], -1.0F, 1.0F, SERVO_DEF_DC_LEFT_US, SERVO_DEF_DC_RIGHT_US);

    for (uint32_t mixIdx = 0; mixIdx < pProfile->servoMixCount; ++mixIdx) {
        float mixedInput = 0.0F;
        switch (pServoMix[mixIdx].inputIndex) {
        case eSERVO_MIX_INPUT_PID_ROLL: mixedInput = pidAttitude[AXIS_IDX_ROLL]; break;
        case eSERVO_MIX_INPUT_PID_PITCH: mixedInput = pidAttitude[AXIS_IDX_PITCH]; break;
        case eSERVO_MIX_INPUT_PID_YAW: mixedInput = pidAttitude[AXIS_IDX_YAW]; break;
        case eSERVO_MIX_INPUT_PID_THROTTLE: mixedInput = targetThrottle; break;
        default: break;
        }
        // TODO: Scale mixedInput to appropriate servo output range
        servoOutputs[SERVO_ID_TO_INDEX (pServoMix[mixIdx].targetServo)] += (uint16_t)(mixedInput * 1000.0F);
    }
}

FJ_TESTABLE FJ_INLINE eSTATUS_t Mixer_Init_ (eMIXER_PROFILE_ID_t profileId, Mixer_t* pOutMixer) {

    pOutMixer->currentProfileId  = profileId;
    pOutMixer->motorsMinThrottle = 0.0F;
    pOutMixer->motorsMaxThrottle = 1.0F;

    return eSTATUS_SUCCESS;
}


eSTATUS_t Mixer_Init (void) {

    return Mixer_Init_ (eMIXER_PROFILE_TILT_ROTOR, &e_Mixer);
}

eSTATUS_t Mixer_Mix (uint32_t usCurrentTime) {

    // clang-format off
    Mixer_MixMotors (&e_Mixer, PidData_Get ()->pidAttitude, FlightData_Get()->targetThrottle, e_Mixer.motorOutputs);
    Mixer_MixServos (&e_Mixer, PidData_Get ()->pidAttitude, FlightData_Get()->targetThrottle, e_Mixer.servoOutputs);
    // clang-format on
    return eSTATUS_SUCCESS;
}
