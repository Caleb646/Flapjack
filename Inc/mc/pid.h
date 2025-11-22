#ifndef MC_PID_H
#define MC_PID_H

#include "conf/board.h"
#include "conf/conf.h"
#include "control.h"
#include "core/core.h"
#include "device/motor/motor.h"
#include "device/servo/servo.h"
#include "fcstate.h"
#include "hal.h"
#include "mc/dshot.h"
#include "peripheral/dma.h"
#include "peripheral/timer.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>


typedef struct {
    float rollP, rollI, rollD;
    float pitchP, pitchI, pitchD;
    float yawP, yawI, yawD;
    float integralLimit;
} PIDInitConf_t;

typedef struct {
    float rollP, rollI, rollD;
    float pitchP, pitchI, pitchD;
    float yawP, yawI, yawD;
    float integralLimit;
    Vec3f prevError;
    Vec3f prevIntegral;
    uint32_t msLastUpdateTime;
    bool isInitialized;
} PID_t;

typedef PID_t vPID_t;

// clang-format off

eSTATUS_t PIDInit (PIDInitConf_t conf);
eSTATUS_t PIDStart (vPID_t* pPID);
eSTATUS_t PIDStop (vPID_t* pPID);
eSTATUS_t PID_Update (vPID_t* pPID, Vec3f const* currentAttitude, Vec3f const* targetAttitude, Vec3f const* maxAttitude, float dt, Vec3f* pOut);
vPID_t const* PIDGetActivePID (void);
vPID_t * PID_GetMutableActivePID (void);

// clang-format on

#define PID_INIT(pSTATUS)                                 \
    do {                                                  \
        PIDInitConf_t conf = { 0 };                       \
        conf.rollI         = PID_STARTING_ROLL_I;         \
        conf.rollD         = PID_STARTING_ROLL_D;         \
        conf.pitchP        = PID_STARTING_PITCH_P;        \
        conf.pitchI        = PID_STARTING_PITCH_I;        \
        conf.pitchD        = PID_STARTING_PITCH_D;        \
        conf.yawP          = PID_STARTING_YAW_P;          \
        conf.yawI          = PID_STARTING_YAW_I;          \
        conf.yawD          = PID_STARTING_YAW_D;          \
        conf.integralLimit = PID_STARTING_INTEGRAL_LIMIT; \
        *(pSTATUS)         = PIDInit (conf);              \
    } while (0)


#endif // MC_PID_H