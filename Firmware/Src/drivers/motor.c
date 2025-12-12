#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "core/core.h"

#include "drivers/dma.h"
#include "drivers/dma_defs.h"
#include "drivers/motor.h"
#include "drivers/tim.h"
#include "drivers/tim_defs.h"

#include "drivers/io/gpio.h"
#include "drivers/io/gpio_defs.h"

#include "cfg/motor.h"

#include "platform/platform.h"

#include "targets/target.h"


typedef struct ServoDesc_s ServoDesc_t;

typedef struct MotorDesc_s {
    Vec3f* pMixes;
    float* pThrottle;
    ServoDesc_t* pLinkedServo;
} MotorDesc_t;

typedef struct Motors_s {
    MotorProtVtbl_t vtbl;
    float outputs[TARG_MAX_MOTORS];
    MotorDesc_t descs[TARG_MAX_MOTORS];
} Motors_t;

typedef struct ServoDesc_s {
    Vec3f* pMixes;
    float* pAngle;
    float* pMaxAngleDeg;
    bool isLinked;
} ServoDesc_t;

typedef struct Servos_s {
    MotorProtVtbl_t vtbl;
    float outputs[TARG_MAX_SERVOS];
    ServoDesc_t descs[TARG_MAX_SERVOS];
} Servos_t;

static TARG_SHARED_MEM_BSS_SECTION Motors_t g_Motors = { 0 };
static TARG_SHARED_MEM_BSS_SECTION Servos_t g_Servos = { 0 };

extern eSTATUS_t Plat_Dshot_Init (MotorsCfg_t const* pCfg, MotorProtVtbl_t* pOutVtbl);

static eSTATUS_t Dshot_Init (MotorsCfg_t const* pCfg, MotorProtVtbl_t* pOutVtbl) {
    return Plat_Dshot_Init (pCfg, pOutVtbl);
}

static void Motor_MixServo (MotorDesc_t* pMotor, Vec3f* const pPidAttitude, float targetThrottle) {

    ServoDesc_t* pServo = pMotor->pLinkedServo;
    if (!pServo) {
        return;
    }

    float mPitchMix     = pMotor->pMixes->pitch;
    float mYawMix       = pMotor->pMixes->yaw;
    float mRollMix      = pMotor->pMixes->roll;
    float mixedThrottle = targetThrottle; // between 0 and 1
    /*
     * NOTE: A PID pitch value should always increase the throttle of both
     * motors regardless of the sign of the PID pitch value.
     */
    mixedThrottle += mPitchMix * ABS_F32 (pPidAttitude->pitch) + mRollMix * pPidAttitude->roll +
                     mYawMix * pPidAttitude->yaw;
    mixedThrottle        = clipf32 (mixedThrottle, 0.0F, 1.0F);
    *(pMotor->pThrottle) = mixedThrottle;

    /*
     *  Servo_t Mixing
     */
    float sPitchMix = pServo->pMixes->pitch;
    float sYawMix   = pServo->pMixes->yaw;
    float sRollMix  = pServo->pMixes->roll;
    float mixedAngle =
    sPitchMix * pPidAttitude->pitch + sRollMix * pPidAttitude->roll + sYawMix * pPidAttitude->yaw;
    *(pServo->pAngle) = (*(pServo->pMaxAngleDeg)) * clipf32 (mixedAngle, -1.0F, 1.0F);
    // NOTE: Maybe Roll should have a negative impact on target angle.
    // Meaning the magnitude of the target angle is closer to 0 the larger
    // pid roll is.
}

eSTATUS_t Motors_Init (void) {

    eSTATUS_t status        = eSTATUS_FAILURE;
    Servos_t* pServos       = &g_Servos;
    Motors_t* pMotors       = &g_Motors;
    MotorsCfg_t const* pCfg = MotorsCfg_Get ();

    if (MOTOR_PROT_IS_DSHOT (pCfg->protType)) {
        status = Dshot_Init (pCfg, &pMotors->vtbl);
    }
    RETURN_IF (FJ_FAIL (status), status, "Failed to initialize motor protocol");

    for (uint32_t i = 0; i < pCfg->nMotors; ++i) {
        pMotors->descs[i].pMixes    = &pCfg->mixes[i];
        pMotors->descs[i].pThrottle = &pMotors->outputs[i];
        if (pCfg->linkedServoIds[i] != eSERVO_ID_NULL) {
            pMotors->descs[i].pLinkedServo = &pServos->descs[SERVO_ID_TO_INDEX (pCfg->linkedServoIds[i])];
        }
    }

    return status;
}

eSTATUS_t Motors_Arm (void) {
    // Implementation of Motors_Arm function
    // ...
    return eSTATUS_SUCCESS;
}

eSTATUS_t Motors_Mix (Vec3f const* pPidAttitude, float targetThrottle) {

    MotorsCfg_t const* pCfg = MotorsCfg_Get ();
    Motors_t* pMotors       = &g_Motors;

    for (uint32_t i = 0; i < pCfg->nMotors; ++i) {
        MotorDesc_t* pMotorDesc = &pMotors->descs[i];
        if (pMotorDesc->pLinkedServo) {
            Motor_MixServo (pMotorDesc, (Vec3f*)pPidAttitude, targetThrottle);
        } else {
            float mPitchMix     = pMotorDesc->pMixes->pitch;
            float mYawMix       = pMotorDesc->pMixes->yaw;
            float mRollMix      = pMotorDesc->pMixes->roll;
            float mixedThrottle = targetThrottle;
            mixedThrottle += mPitchMix * ABS_F32 (pPidAttitude->pitch) +
                             mRollMix * pPidAttitude->roll + mYawMix * pPidAttitude->yaw;
            mixedThrottle            = clipf32 (mixedThrottle, 0.0F, 1.0F);
            *(pMotorDesc->pThrottle) = mixedThrottle;
        }
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t Motors_Update (void) {

    Motors_t* pMotors = &g_Motors;
    if (!pMotors->vtbl.fnUpdateMotors) {
        return eSTATUS_FAILURE;
    }
    return pMotors->vtbl.fnUpdateMotors (pMotors->outputs, MotorsCfg_Get ()->nMotors);
}


eSTATUS_t Servos_Init (void) {

    eSTATUS_t status        = eSTATUS_FAILURE;
    Servos_t* pServos       = &g_Servos;
    ServosCfg_t const* pCfg = ServosCfg_Get ();


    for (uint32_t i = 0; i < pCfg->nServos; ++i) {
        pServos->descs[i].pMixes       = &pCfg->mixes[i];
        pServos->descs[i].pAngle       = &pServos->outputs[i];
        pServos->descs[i].pMaxAngleDeg = &pCfg->maxAngleDeg;
        pServos->descs[i].isLinked     = (pCfg->linkedMotorIds[i] != eMOTOR_ID_NULL);
    }

    return status;
}