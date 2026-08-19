#ifndef MC_MIXER_H
#define MC_MIXER_H

#include <stdbool.h>
#include <stdint.h>

#include "core/core.h"

#include "target.h"

typedef uint8_t eMOTOR_ID_t;
enum {
    eMOTOR_ID_NONE = 0,

    eMOTOR_1_ID,
    eMOTOR_LEFT_ID = eMOTOR_1_ID,

    eMOTOR_2_ID,
    eMOTOR_RIGHT_ID = eMOTOR_2_ID,

    eMOTOR_3_ID,
    eMOTOR_4_ID,
    eMOTOR_5_ID,
    eMOTOR_6_ID,
    eMOTOR_7_ID,
    eMOTOR_8_ID
};

#define MOTOR_ID_TO_IDX(MOTOR_ID) ((MOTOR_ID) - 1U)

typedef uint8_t eSERVO_ID_t;
enum {
    eSERVO_ID_NONE = 0,

    eSERVO_1_ID,
    eSERVO_LEFT_MOTOR_ID = eSERVO_1_ID,

    eSERVO_2_ID,
    eSERVO_RIGHT_MOTOR_ID = eSERVO_2_ID,

    eSERVO_3_ID,
    eSERVO_LEFT_AILERON_ID = eSERVO_3_ID,

    eSERVO_4_ID,
    eSERVO_RIGHT_AILERON_ID = eSERVO_4_ID,

    eSERVO_5_ID,
    eSERVO_RUDDER_ID = eSERVO_5_ID,

    eSERVO_6_ID,
    eSERVO_ELEVATOR_ID = eSERVO_6_ID,

    eSERVO_7_ID,
    eSERVO_8_ID
};

#define SERVO_ID_TO_IDX(SERVO_ID) ((SERVO_ID) - 1U)

typedef struct MotorMix_s {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} MotorMix_t;

typedef uint8_t eSERVO_MIX_INPUT_IDX_t;
enum {
    eSERVO_MIX_INPUT_PID_ROLL,
    eSERVO_MIX_INPUT_PID_PITCH,
    eSERVO_MIX_INPUT_PID_YAW,
    eSERVO_MIX_INPUT_PID_THROTTLE,
    eSERVO_MIX_INPUT_COUNT

};

typedef struct ServoMix_s {
    eSERVO_ID_t targetServo;
    eSERVO_MIX_INPUT_IDX_t inputIndex;
    /* Signed contribution of this input to the servo, in normalised servo travel
     * (+1.0 = full deflection towards SERVO_RIGHT_US_DC). Mixes that need a pair
     * of surfaces to move oppositely (differential tilt, ailerons) do it here. */
    float weight;
} ServoMix_t;

typedef uint8_t eMIXER_PROFILE_ID_t;
enum {
    eMIXER_PROFILE_TEST_SETUP,
    eMIXER_PROFILE_TILT_ROTOR,
    eMIXER_PROFILE_AIRPLANE,
    eMIXER_PROFILE_COUNT
};

typedef struct MixerProfile_s {
    uint8_t motorCount;
    uint8_t servoCount;
    MotorMix_t const* pMotorMix;
    ServoMix_t const* pServoMix;
    uint8_t servoMixCount;
} MixerProfile_t;

typedef struct Mixer_s {
    MixerProfile_t* pCurrentProfile;
    MixerProfile_t profiles[eMIXER_PROFILE_COUNT];
    float motorsMinThrottle;
    float motorsMaxThrottle;
    float servoTravelLimit;
    float motorOutputs[BRD_MOTOR_COUNT];
    uint16_t servoOutputs[BRD_SERVO_COUNT];
} Mixer_t;

FJ_DECLARE_SHARED (MotorMix_t, g_TestMotorMix[]);
FJ_DECLARE_SHARED (MotorMix_t, g_TiltMotorMix[]);
FJ_DECLARE_SHARED (MotorMix_t, g_AirplaneMotorMix[]);

FJ_DECLARE_SHARED (ServoMix_t, g_TestServoMix[]);
FJ_DECLARE_SHARED (ServoMix_t, g_TiltServoMix[]);
FJ_DECLARE_SHARED (ServoMix_t, g_AirplaneServoMix[]);

FJ_DECLARE_SHARED (Mixer_t, g_Mixer);

eSTATUS_t Mixer_Init (void);
void Mixer_MixMotors (Mixer_t* pMixer, float const pidData[AXIS_IDX_COUNT], float motorOutputs[BRD_MOTOR_COUNT]);
void Mixer_MixServos (Mixer_t* pMixer, float const pidData[AXIS_IDX_COUNT], uint16_t servoOutputs[BRD_SERVO_COUNT]);


#endif // MC_MIXER_H