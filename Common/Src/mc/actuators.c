#include "mc/actuators.h"
#include "common.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "conf/ids.h"
#include "device/motor/motor.h"
#include "device/servo/servo.h"
#include "log/logger.h"
#include "mc/dshot.h"
#include "mem/vector.h"
#include "peripheral/gpio.h"
#include "peripheral/timer.h"
#include <string.h>


#ifndef UNIT_TEST
// static eSTATUS_t
// ActuatorsMixPair (Servo_t* pServo, Motor_t* pMotor, Vec3f pidAttitude, float targetThrottle);
#endif // UNIT_TEST

/*
 * \param pidAttitude roll, pitch, and yaw are between -1 and 1
 */
STATIC_TESTABLE_DECL eSTATUS_t
ActuatorsMixPair (Servo_t* pServo, Motor_t* pMotor, Vec3f pidAttitude, float targetThrottle) {
    /*
     *  Motor_t Mixing
     */
    float mPitchMix     = pMotor->pitchMix;
    float mYawMix       = pMotor->yawMix;
    float mRollMix      = pMotor->rollMix;
    float mixedThrottle = targetThrottle; // between 0 and 1
    /*
     * NOTE: A PID pitch value should always increase the throttle of both
     * motors regardless of the sign of the PID pitch value.
     */
    mixedThrottle += mPitchMix * ABS_F32 (pidAttitude.pitch) +
                     mRollMix * pidAttitude.roll + mYawMix * pidAttitude.yaw;
    mixedThrottle = clipf32 (mixedThrottle, 0.0F, 1.0F);

    /*
     *  Servo_t Mixing
     */
    float sPitchMix  = pServo->pitchMix;
    float sYawMix    = pServo->yawMix;
    float sRollMix   = pServo->rollMix;
    float mixedAngle = sPitchMix * pidAttitude.pitch +
                       sRollMix * pidAttitude.roll + sYawMix * pidAttitude.yaw;
    // NOTE: Maybe Roll should have a negative impact on target angle.
    // Meaning the magnitude of the target angle is closer to 0 the larger
    // pid roll is.
    mixedAngle = clipf32 (mixedAngle, -1.0F, 1.0F) * pServo->maxAngle;

    /* Motor_t throttle should be between 0 and 1 */
    MotorWrite (pMotor, mixedThrottle);
    ServoWrite (pServo, mixedAngle);
    return eSTATUS_SUCCESS;
}

/*
 * \brief After motors are armed, a motor write has to be issued at
 * least every 5ms or the motor ESC will stop the motor.
 */
STATIC_TESTABLE_DECL eSTATUS_t ActuatorsArmMotor (Motor_t* pMotor) {

    // #ifndef USE_SERVOS_ONLY

    uint32_t msDelay    = 2;
    uint32_t msMaxTime  = 350;
    uint32_t iterations = msMaxTime / msDelay;
    for (uint32_t i = 0; i < iterations; ++i) {
        /* NOTE: A DShot value of all 0s is a special command to
         * the esc to arm/disarm the motor depending on the esc's current state.
         * The reason MotorWrite isn't used is because it uses a valid throttle value between > 48 and < 2048 */
        if (MotorWriteCmd (pMotor, eMOTOR_CMD_ARM) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to arm motor ID %u", pMotor->motorId);
            return eSTATUS_FAILURE;
        }
        // NOTE: assumes DShot150 is used.
        vTaskDelay (pdMS_TO_TICKS (msDelay));
    }

    // // Slowly increase the throttle to 15%
    // msDelay              = 4;
    // msMaxTime            = 3000;
    // float i              = MOTOR_MIN_THROTTLE;
    // float targetThrottle = MOTOR_STARTUP_THROTTLE;
    // float increment      = targetThrottle / (float)(msMaxTime /
    // msDelay); while (i < targetThrottle) {
    //     for (uint32_t motorIdx = 0; motorIdx < numMotors; ++motorIdx) {

    //         Motor_t* pMotor = &pMotors[motorIdx];
    //         if (MotorWrite (pMotor, i) != eSTATUS_SUCCESS) {
    //             LOG_ERROR ("Failed to arm motor ID %u",
    //             pMotor->motorId); return eSTATUS_FAILURE;
    //         }
    //     }
    //     vTaskDelay (pdMS_TO_TICKS (msDelay));
    //     i += increment;
    // }

    return eSTATUS_SUCCESS;
}

eSTATUS_t ActuatorsInitMotor (MotorInitConf_t conf) {

    eSTATUS_t status           = MotorInit (conf);
    MotorBoardConf_t boardConf = conf.boardConf;
    ServoBoardConf_t* pLinkedServoBoardConf = boardConf.pLinkedServoBoardConf;
    eDEVICE_ID_t motorId = boardConf.motorId;
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize motor");
        return status;
    }

    Motor_t* pMotor = MotorGetById (motorId);
    if (pLinkedServoBoardConf != NULL) {
        eDEVICE_ID_t linkedServoId = pLinkedServoBoardConf->servoId;
        pMotor->linkedServoId      = linkedServoId;
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t ActuatorsInitServo (ServoInitConf_t conf) {

    eSTATUS_t status           = ServoInit (conf);
    ServoBoardConf_t boardConf = conf.boardConf;
    MotorBoardConf_t* pLinkedMotorBoardConf = boardConf.pLinkedMotorBoardConf;
    eDEVICE_ID_t servoId = boardConf.servoId;
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize servo");
        return status;
    }

    Servo_t* pServo = ServoGetById (servoId);
    if (pLinkedMotorBoardConf != NULL) {
        eDEVICE_ID_t linkedMotorId = pLinkedMotorBoardConf->motorId;
        pServo->linkedMotorId      = linkedMotorId;
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t
ActuatorsInit (MotorBoardConf_t* pMotorBoardConfs, uint32_t numMotors, ServoBoardConf_t* pServoBoardConfs, uint32_t numServos) {

    if (pMotorBoardConfs == NULL && pServoBoardConfs == NULL) {
        LOG_ERROR ("No motor or servo board configurations provided");
        return eSTATUS_FAILURE;
    }

    for (uint32_t i = 0; i < numMotors; ++i) {

        eSTATUS_t status                = eSTATUS_SUCCESS;
        MotorBoardConf_t motorBoardConf = pMotorBoardConfs[i];

        ACTUATORS_INIT_MOTOR (&status, motorBoardConf);
        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to init motor %d", motorBoardConf.motorId);
            return status;
        }
    }

    for (uint32_t i = 0; i < numServos; ++i) {

        eSTATUS_t status                = eSTATUS_SUCCESS;
        ServoBoardConf_t servoBoardConf = pServoBoardConfs[i];

        ACTUATORS_INIT_SERVO (&status, servoBoardConf);
        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to init servo %d", servoBoardConf.servoId);
            return status;
        }
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t ActuatorsStart (void) {

    VECTOR_FOR_EACH (ServoGetAll (), Servo_t, ServoStart);
    VECTOR_FOR_EACH (MotorGetAll (), Motor_t, MotorStart);
    VECTOR_FOR_EACH (MotorGetAll (), Motor_t, ActuatorsArmMotor);
    return eSTATUS_SUCCESS;
}

eSTATUS_t ActuatorsStop (void) {

    VECTOR_FOR_EACH (ServoGetAll (), Servo_t, ServoStop);
    VECTOR_FOR_EACH (MotorGetAll (), Motor_t, MotorStop);
    return eSTATUS_SUCCESS;
}

eSTATUS_t ActuatorsWrite (Vec3f pidAttitude, float targetThrottle) {

    eSTATUS_t status       = eSTATUS_SUCCESS;
    Vector_t* pMotorVector = MotorGetAll ();
    if (pMotorVector == NULL) {
        LOG_ERROR ("No motors available");
        return eSTATUS_FAILURE;
    }

    for (uint32_t i = 0; i < pMotorVector->size; ++i) {

        Motor_t* pMotor = (Motor_t*)Vector_At (pMotorVector, i);
        Servo_t* pServo = ServoGetById (pMotor->linkedServoId);
        // If the motor has a linked servo, mix and write to both the motor and servo
        if (pServo != NULL) {
            status = ActuatorsMixPair (pServo, pMotor, pidAttitude, targetThrottle);
            if (status != eSTATUS_SUCCESS) {
                LOG_ERROR (
                "Failed to mix motor ID %u and servo ID %u",
                pMotor->motorId,
                pServo->servoId
                );
                return eSTATUS_FAILURE;
            }
        } else {
            // No linked servo, just mix and write to the motor
            float mPitchMix     = pMotor->pitchMix;
            float mYawMix       = pMotor->yawMix;
            float mRollMix      = pMotor->rollMix;
            float mixedThrottle = targetThrottle; // between 0 and 1
            /*
             * NOTE: A PID pitch value should always increase the throttle of both
             * motors regardless of the sign of the PID pitch value.
             */
            mixedThrottle += mPitchMix * ABS_F32 (pidAttitude.pitch) +
                             mRollMix * pidAttitude.roll +
                             mYawMix * pidAttitude.yaw;
            mixedThrottle = clipf32 (mixedThrottle, 0.0F, 1.0F);
            status        = MotorWrite (pMotor, mixedThrottle);
            if (status != eSTATUS_SUCCESS) {
                LOG_ERROR ("Failed to write motor ID %u", pMotor->motorId);
                return eSTATUS_FAILURE;
            }
        }
    }

    Vector_t* pServoVector = ServoGetAll ();
    if (pServoVector == NULL) {
        LOG_ERROR ("No servos available");
        return eSTATUS_FAILURE;
    }

    for (uint32_t i = 0; i < pServoVector->size; ++i) {

        Servo_t* pServo = (Servo_t*)Vector_At (pServoVector, i);
        // Only write to servos that are not linked to a motor
        // as those servos were already written to in the motor loop above.
        if (pServo->linkedMotorId == eDEVICE_ID_NULL) {

            float sPitchMix  = pServo->pitchMix;
            float sYawMix    = pServo->yawMix;
            float sRollMix   = pServo->rollMix;
            float mixedAngle = sPitchMix * pidAttitude.pitch +
                               sRollMix * pidAttitude.roll +
                               sYawMix * pidAttitude.yaw;
            mixedAngle = clipf32 (mixedAngle, -pServo->maxAngle, pServo->maxAngle);
            status = ServoWrite (pServo, mixedAngle);
            if (status != eSTATUS_SUCCESS) {
                LOG_ERROR ("Failed to write servo ID %u", pServo->servoId);
                return eSTATUS_FAILURE;
            }
        }
    }
    return eSTATUS_SUCCESS;
}


void ActuatorsLogData (void) {
    // LOG_DATA_ACTUATORS_DATA ("Left Motor_t", gLeftMotor, "Left Servo_t", gLeftServo);
}