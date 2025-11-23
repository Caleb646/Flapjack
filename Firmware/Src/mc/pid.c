#include "mc/pid.h"
#include "common.h"
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



#define PID_VALID(pPID) ((pPID) != NULL && (pPID)->isInitialized == true)

static SHARED_MEM_SECTION vPID_t gPID = { 0 };

eSTATUS_t PIDInit (PIDInitConf_t conf) {

    vPID_t* pPID = &gPID;
    memset ((void*)pPID, 0, sizeof (vPID_t));
    pPID->rollP            = conf.rollP;
    pPID->rollI            = conf.rollI;
    pPID->rollD            = conf.rollD;
    pPID->pitchP           = conf.pitchP;
    pPID->pitchI           = conf.pitchI;
    pPID->pitchD           = conf.pitchD;
    pPID->yawP             = conf.yawP;
    pPID->yawI             = conf.yawI;
    pPID->yawD             = conf.yawD;
    pPID->integralLimit    = conf.integralLimit;
    pPID->msLastUpdateTime = GetMilliseconds ();
    pPID->isInitialized    = true;
    return eSTATUS_SUCCESS;
}

eSTATUS_t PIDStart (vPID_t* pPID) {

    if (PID_VALID (pPID) == false) {
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t PIDStop (vPID_t* pPID) {

    if (PID_VALID (pPID) == false) {
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t
PID_Update (vPID_t* pPID, Vec3f const* pCurrentAttitude, Vec3f const* pTargetAttitude, Vec3f const* pMaxAttitude, float dt, Vec3f* pOut) {

    if (PID_VALID (pPID) == false || pOut == NULL || pCurrentAttitude == NULL ||
        pTargetAttitude == NULL || pMaxAttitude == NULL) {
        return eSTATUS_FAILURE;
    }

    Vec3f currentAttitude = *pCurrentAttitude;
    Vec3f targetAttitude  = *pTargetAttitude;
    Vec3f maxAttitude     = *pMaxAttitude;

    float P         = pPID->rollP;
    float I         = pPID->rollI;
    float D         = pPID->rollD;
    float rollError = targetAttitude.roll - currentAttitude.roll;
    float rollIntegral =
    clipf32 (pPID->prevIntegral.roll + rollError * dt, -pPID->integralLimit, pPID->integralLimit);
    float rollDerivative = (rollError - pPID->prevError.roll) / dt;
    // pOutputPIDAttitude->roll = 0.01f * (P * rollError + I * rollIntegral - D * rollDerivative);

    // Scale PID output between -1 and 1
    pOut->roll =
    clipf32 ((P * rollError + I * rollIntegral - D * rollDerivative), -maxAttitude.roll, maxAttitude.roll) /
    maxAttitude.roll;

    P                = pPID->pitchP;
    I                = pPID->pitchI;
    D                = pPID->pitchD;
    float pitchError = targetAttitude.pitch - currentAttitude.pitch;
    float pitchIntegral =
    clipf32 (pPID->prevIntegral.pitch + pitchError * dt, -pPID->integralLimit, pPID->integralLimit);
    float pitchDerivative = (pitchError - pPID->prevError.pitch) / dt;
    // pOutputPIDAttitude->pitch = 0.01f * (P * pitchError + I * pitchIntegral - D * pitchDerivative);

    // Scale PID output between -1 and 1
    pOut->pitch = clipf32 (
                  (P * pitchError + I * pitchIntegral - D * pitchDerivative),
                  -maxAttitude.pitch,
                  maxAttitude.pitch
                  ) /
                  maxAttitude.pitch;

    P              = pPID->yawP;
    I              = pPID->yawI;
    D              = pPID->yawD;
    float yawError = targetAttitude.yaw - currentAttitude.yaw;
    float yawIntegral =
    clipf32 (pPID->prevIntegral.yaw + yawError * dt, -pPID->integralLimit, pPID->integralLimit);
    float yawDerivative = (yawError - pPID->prevError.yaw) / dt;
    // pOutputPIDAttitude->yaw = 0.01f * (P * yawError + I * yawIntegral - D * yawDerivative);

    // Scale PID output between -1 and 1
    pOut->yaw =
    clipf32 ((P * yawError + I * yawIntegral - D * yawDerivative), -maxAttitude.yaw, maxAttitude.yaw) /
    maxAttitude.yaw;

    pPID->prevIntegral.roll  = rollIntegral;
    pPID->prevIntegral.pitch = pitchIntegral;
    pPID->prevIntegral.yaw   = yawIntegral;

    pPID->prevError.roll  = rollError;
    pPID->prevError.pitch = pitchError;
    pPID->prevError.yaw   = yawError;

    pPID->msLastUpdateTime = GetMilliseconds ();
    return eSTATUS_SUCCESS;
}

vPID_t const* PIDGetActivePID (void) {

    if (PID_VALID (&gPID) == false) {
        return NULL;
    }
    return &gPID;
}

vPID_t* PID_GetMutableActivePID (void) {

    if (PID_VALID (&gPID) == false) {
        return NULL;
    }
    return &gPID;
}