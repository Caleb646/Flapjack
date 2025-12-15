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


typedef struct Pwm_s {
    TimChannel_t* pTimChans;
    uint8_t nChans;
} Pwm_t;

CFG_DEFINE (MotorsCfg_t, MotorsCfg);
CFG_DEFINE (ServosCfg_t, ServosCfg);
static TARG_SHARED_MEM_BSS_SECTION Pwm_t g_Pwm             = { 0 };
static TARG_SHARED_MEM_BSS_SECTION MotorsDevice_t g_Motors = { 0 };
static TARG_SHARED_MEM_BSS_SECTION ServosDevice_t g_Servos = { 0 };

extern eSTATUS_t Plat_Dshot_Init (MotorsCfg_t const* pCfg, MotorProtVtbl_t* pOutVtbl);

static eSTATUS_t Dshot_Init (MotorsCfg_t const* pCfg, MotorProtVtbl_t* pOutVtbl) {
    return Plat_Dshot_Init (pCfg, pOutVtbl);
}

static void Motor_MixServo (MotorView_t* pMotor, Vec3f* const pPidAttitude, float targetThrottle) {

    ServoView_t* pServo = pMotor->pLinkedServo;
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
    ServosDevice_t* pServos = &g_Servos;
    MotorsDevice_t* pMotors = &g_Motors;
    MotorsCfg_t const* pCfg = MotorsCfg_Get ();

    if (MOTOR_PROT_IS_DSHOT (pCfg->protType)) {
        status = Dshot_Init (pCfg, &pMotors->vtbl);
    }
    RETURN_IF (FJ_FAIL (status), status, "Failed to initialize motor protocol");

    for (uint32_t i = 0; i < pCfg->nMotors; ++i) {
        pMotors->views[i].pMixes    = &pCfg->mixes[i];
        pMotors->views[i].pThrottle = &pMotors->outputs[i];
        if (pCfg->linkedServoIds[i] != eSERVO_ID_NULL) {
            pMotors->views[i].pLinkedServo = &pServos->views[SERVO_ID_TO_INDEX (pCfg->linkedServoIds[i])];
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
    MotorsDevice_t* pMotors = &g_Motors;

    for (uint32_t i = 0; i < pCfg->nMotors; ++i) {
        MotorView_t* pMotorDesc = &pMotors->views[i];
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

    MotorsDevice_t* pMotors = &g_Motors;
    if (!pMotors->vtbl.fnUpdateMotors) {
        return eSTATUS_FAILURE;
    }
    return pMotors->vtbl.fnUpdateMotors (pMotors->outputs, MotorsCfg_Get ()->nMotors);
}

static eSTATUS_t Pwm_Update (float const* pAngles, uint32_t nAngles) {

    ServosCfg_t const* pServosCfg = ServosCfg_Get ();
    Pwm_t* pPwm                   = &g_Pwm;
    if (!pAngles || nAngles == 0U || nAngles > pPwm->nChans) {
        return eSTATUS_INVALID_ARG;
    }

    float angleDeg       = 0U;
    float startAngle     = 0.0F;
    float endAngle       = 0.0F;
    float startUs        = 0.0F;
    float endUs          = 0.0F;
    float const maxAngle = pServosCfg->maxAngleDeg;

    for (uint32_t servoIdx = 0; servoIdx < nAngles; ++servoIdx) {
        angleDeg   = pAngles[servoIdx];
        startAngle = -maxAngle;
        endAngle   = 0.0F;
        startUs    = pServosCfg->dcLeftUs;
        endUs      = pServosCfg->dcMiddleUs;


        if (angleDeg > 0) {
            startAngle = 0.0F;
            endAngle   = maxAngle;
            startUs    = pServosCfg->dcMiddleUs;
            endUs      = pServosCfg->dcRightUs;
        }

        float pulseUs = mapf32 (angleDeg, startAngle, endAngle, startUs, endUs);
        TimChan_SetCC (&pPwm->pTimChans[servoIdx], (uint32_t)pulseUs);
    }

    return eSTATUS_SUCCESS;
}

static eSTATUS_t Pwm_Init (ServosCfg_t const* pCfg) {

    eSTATUS_t status = eSTATUS_SUCCESS;
    Pwm_t* pPwm      = &g_Pwm;
    pPwm->nChans     = pCfg->nServos;
    pPwm->pTimChans  = Alloc_SharedMem (sizeof (TimChannel_t) * pCfg->nServos);
    if (!pPwm->pTimChans) {
        return eSTATUS_FAILURE;
    }

    for (uint32_t i = 0; i < pPwm->nChans; ++i) {
        status = TimChan_InitCC (pCfg->gpios[i], &pPwm->pTimChans[i]);
        TimDev_SetPWMPeriod (pPwm->pTimChans[i].pTimBaseDev, ONE_MHZ, SERVO_DEF_PERIOD_HZ);
        TimChan_SetCC (&pPwm->pTimChans[i], 0U);
        TimChan_Start (&pPwm->pTimChans[i], NULL, 0U);
        RETURN_IF (FJ_FAIL (status), status, "Failed to initialize TIM channel for servo %u", i);
    }
    return eSTATUS_SUCCESS;
}


eSTATUS_t Servos_Init (void) {

    ServosDevice_t* pServos = &g_Servos;
    ServosCfg_t const* pCfg = ServosCfg_Get ();

    for (uint32_t i = 0; i < pCfg->nServos; ++i) {
        pServos->views[i].pMixes       = &pCfg->mixes[i];
        pServos->views[i].pAngle       = &pServos->outputs[i];
        pServos->views[i].pMaxAngleDeg = &pCfg->maxAngleDeg;
        pServos->views[i].isLinked     = (pCfg->linkedMotorIds[i] != eMOTOR_ID_NULL);
    }

    return Pwm_Init (pCfg);
}

eSTATUS_t Servos_Mix (Vec3f const* pPidAttitude, Vec3f const* pTargetAttitude) {

    if (!pPidAttitude || !pTargetAttitude) {
        return eSTATUS_NULL_ARG;
    }

    ServosCfg_t const* pCfg = ServosCfg_Get ();
    ServosDevice_t* pServos = &g_Servos;

    for (uint32_t i = 0; i < pCfg->nServos; ++i) {
        ServoView_t* pServoDesc = &pServos->views[i];
        if (pServoDesc->isLinked) {
            continue;
        }

        float sPitchMix = pServoDesc->pMixes->pitch;
        float sYawMix   = pServoDesc->pMixes->yaw;
        float sRollMix  = pServoDesc->pMixes->roll;
        float mixedAngle =
        sPitchMix * pPidAttitude->pitch + sRollMix * pPidAttitude->roll + sYawMix * pPidAttitude->yaw;
        *(pServoDesc->pAngle) = (*(pServoDesc->pMaxAngleDeg)) * clipf32 (mixedAngle, -1.0F, 1.0F);
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t Servos_Update (void) {

    ServosDevice_t* pServos = &g_Servos;

    if (!pServos->vtbl.fnUpdateServos) {
        return eSTATUS_FAILURE;
    }

    pServos->vtbl.fnUpdateServos (pServos->outputs, ServosCfg_Get ()->nServos);
    return eSTATUS_SUCCESS;
}