#ifndef AERO_MIXER_H
#define AERO_MIXER_H

#include "common.h"

typedef struct MotorMix_s {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} MotorMix_t;

typedef uint8_t eSERVO_MIX_INPUT_ID_t;
enum {
    eSERVO_MIX_INPUT_PID_ROLL,
    eSERVO_MIX_INPUT_PID_PITCH,
    eSERVO_MIX_INPUT_PID_YAW,
    eSERVO_MIX_INPUT_PID_THROTTLE,
    eSERVO_MIX_INPUT_COUNT

};

typedef struct ServoMix_s {
    eSERVO_ID_t targetServo;
    eSERVO_MIX_INPUT_ID_t inputIndex;
} ServoMix_t;

typedef uint8_t eMIXER_PROFILE_ID_t;
enum { eMIXER_PROFILE_TILT_ROTOR, eMIXER_PROFILE_AIRPLANE, eMIXER_PROFILE_COUNT };

typedef struct MixerProfile_s {
    uint8_t motorCount;
    uint8_t servoCount;
    MotorMix_t const* pMotorMix;
    ServoMix_t const* pServoMix;
    uint8_t servoMixCount;
} MixerProfile_t;

typedef struct Mixer_s {
    eMIXER_PROFILE_ID_t currentProfileId;
    MixerProfile_t profiles[eMIXER_PROFILE_COUNT];
    float motorsMinThrottle;
    float motorsMaxThrottle;
    float motorOutputs[TARG_MAX_MOTORS];
    uint16_t servoOutputs[TARG_MAX_SERVOS];
} Mixer_t;

FJ_DECLARE_SHARED (MotorMix_t, e_TiltMotorMix[]);
FJ_DECLARE_SHARED (ServoMix_t, e_TiltServoMix[]);
FJ_DECLARE_SHARED (ServoMix_t, e_AirplaneServoMix[]);
FJ_DECLARE_SHARED (Mixer_t, e_Mixer);

eSTATUS_t Mixer_Init (void);
eSTATUS_t Mixer_Mix (void);
eSTATUS_t Mixer_Update (void);

#endif // AERO_MIXER_H