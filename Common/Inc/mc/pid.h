#ifndef MC_PID_H
#define MC_PID_H

#include "common.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "control.h"
#include "device/motor/motor.h"
#include "device/servo/servo.h"
#include "hal.h"
#include "mc/dshot.h"
#include "mc/fcstate.h"
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

eSTATUS_t PIDInit (PIDInitConf_t conf);
eSTATUS_t PIDStart (vPID_t* pPID);
eSTATUS_t PIDStop (vPID_t* pPID);
eSTATUS_t PIDUpdate (
vPID_t* pPID,
Vec3f const* currentAttitude,
Vec3f const* targetAttitude,
Vec3f const* maxAttitude,
Vec3f* pOut
);
vPID_t* PIDGetActivePID (void);

#define PID_INIT(pSTATUS)                                  \
    do {                                                   \
        PIDInitConf_t conf  = { 0 };                       \
        conf.rollI          = PID_STARTING_ROLL_I;         \
        conf.rollD          = PID_STARTING_ROLL_D;         \
        conf.pitchP         = PID_STARTING_PITCH_P;        \
        conf.pitchI         = PID_STARTING_PITCH_I;        \
        conf.pitchD         = PID_STARTING_PITCH_D;        \
        conf.yawP           = PID_STARTING_YAW_P;          \
        conf.yawI           = PID_STARTING_YAW_I;          \
        conf.yawD           = PID_STARTING_YAW_D;          \
        conf.integralLimit  = PID_STARTING_INTEGRAL_LIMIT; \
        conf.prevError.x    = 0.0F;                        \
        conf.prevError.y    = 0.0F;                        \
        conf.prevError.z    = 0.0F;                        \
        conf.prevIntegral.x = 0.0F;                        \
        conf.prevIntegral.y = 0.0F;                        \
        conf.prevIntegral.z = 0.0F;                        \
        *(pSTATUS)          = PIDInit (conf);              \
    } while (0)


#endif // MC_PID_H