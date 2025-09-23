#include "mc/actuators.h"
#include "common.h"
#include "conf/conf.h"
#include "log/logger.h"
#include "mc/dshot.h"
#include "periphs/gpio.h"
#include "periphs/timer.h"
#include <string.h>


// #define CHECK_SERVO_DESCRIPTOR_OK(pServoDesc)                                     \
//     (                                                                             \
//     (pServoDesc) != NULL && (pServoDesc)->usLeftDutyCycle != 0U &&                \
//     (pServoDesc)->usMiddleDutyCycle != 0U && (pServoDesc)->usRightDutyCycle != 0U \
//     )

// #define CHECK_SERVO_OK(pServo) \
//     ((pServo) != NULL && PWM_CHECK_OK (&((pServo)->pwm)))

eSTATUS_t PIDUpdateAttitude (
PIDContext* pidContext,
Vec3f currentAttitude,    // degrees
Vec3f targetAttitude,     // degrees
Vec3f maxAttitude,        // degrees
float dt,                 // seconds
Vec3f* pOutputPIDAttitude // degrees
) {

    if (pidContext == NULL || pOutputPIDAttitude == NULL) {
        LOG_ERROR ("PIDContext or output pointer is NULL");
        return eSTATUS_FAILURE;
    }

    if (dt == 0.0F) {
        LOG_ERROR ("dt cannot be zero");
        return eSTATUS_FAILURE;
    }

    float P            = pidContext->rollP;
    float I            = pidContext->rollI;
    float D            = pidContext->rollD;
    float rollError    = targetAttitude.roll - currentAttitude.roll;
    float rollIntegral = clipf32 (
    pidContext->prevIntegral.roll + rollError * dt,
    -pidContext->integralLimit,
    pidContext->integralLimit
    );
    float rollDerivative = (rollError - pidContext->prevError.roll) / dt;
    // pOutputPIDAttitude->roll = 0.01f * (P * rollError + I * rollIntegral - D * rollDerivative);

    // Scale PID output between -1 and 1
    pOutputPIDAttitude->roll = clipf32 (
                               (P * rollError + I * rollIntegral - D * rollDerivative),
                               -maxAttitude.roll,
                               maxAttitude.roll
                               ) /
                               maxAttitude.roll;

    P                   = pidContext->pitchP;
    I                   = pidContext->pitchI;
    D                   = pidContext->pitchD;
    float pitchError    = targetAttitude.pitch - currentAttitude.pitch;
    float pitchIntegral = clipf32 (
    pidContext->prevIntegral.pitch + pitchError * dt,
    -pidContext->integralLimit,
    pidContext->integralLimit
    );
    float pitchDerivative = (pitchError - pidContext->prevError.pitch) / dt;
    // pOutputPIDAttitude->pitch = 0.01f * (P * pitchError + I * pitchIntegral - D * pitchDerivative);

    // Scale PID output between -1 and 1
    pOutputPIDAttitude->pitch =
    clipf32 (
    (P * pitchError + I * pitchIntegral - D * pitchDerivative),
    -maxAttitude.pitch,
    maxAttitude.pitch
    ) /
    maxAttitude.pitch;

    P                 = pidContext->yawP;
    I                 = pidContext->yawI;
    D                 = pidContext->yawD;
    float yawError    = targetAttitude.yaw - currentAttitude.yaw;
    float yawIntegral = clipf32 (
    pidContext->prevIntegral.yaw + yawError * dt,
    -pidContext->integralLimit,
    pidContext->integralLimit
    );
    float yawDerivative = (yawError - pidContext->prevError.yaw) / dt;
    // pOutputPIDAttitude->yaw = 0.01f * (P * yawError + I * yawIntegral - D * yawDerivative);

    // Scale PID output between -1 and 1
    pOutputPIDAttitude->yaw = clipf32 (
                              (P * yawError + I * yawIntegral - D * yawDerivative),
                              -maxAttitude.yaw,
                              maxAttitude.yaw
                              ) /
                              maxAttitude.yaw;

    pidContext->prevIntegral.roll  = rollIntegral;
    pidContext->prevIntegral.pitch = pitchIntegral;
    pidContext->prevIntegral.yaw   = yawIntegral;

    pidContext->prevError.roll  = rollError;
    pidContext->prevError.pitch = pitchError;
    pidContext->prevError.yaw   = yawError;

    return eSTATUS_SUCCESS;
}

#ifndef UNIT_TEST

static float ServoAngle2PWM (Servo* pServo, float targetAngle);

#endif // UNIT_TEST

static Servo gServos[eSERVO_ID_MAX] = { 0 };
static Motor gMotors[eMOTOR_ID_MAX] = { 0 };

STATIC_TESTABLE_DECL float ServoAngle2PWM (Servo* pServo, float targetAngle) {
    if (CHECK_SERVO_OK (pServo) != TRUE) {
        LOG_ERROR ("ServoDescriptor is not valid");
        return 0.0F;
    }
    /*
     * NOTE: The servo on its own can move between -maxAngle and +maxAngle. But when placed
     * in the drone, it may only be able to move between -usableMaxAngle and +usableMaxAngle.
     *
     * Clip the target angle to the usable range first, then map it to the PWM duty cycle using the max angle.
     */
    ServoDescriptor* pDesc = &pServo->desc;
    // Handle asymmetric servo ranges: map negative angles to
    // [usLeftDutyCycle, usMiddleDutyCycle], positive to [usMiddleDutyCycle, usRightDutyCycle]
    if (targetAngle < 0) {
        return mapf32 (
        targetAngle,
        -pDesc->maxAngle,
        0.0F,
        (float)pDesc->usLeftDutyCycle,
        (float)pDesc->usMiddleDutyCycle
        );
    }

    if (targetAngle > 0) {
        return mapf32 (
        targetAngle,
        0.0F,
        pDesc->maxAngle,
        (float)pDesc->usMiddleDutyCycle,
        (float)pDesc->usRightDutyCycle
        );
    }

    return (float)pDesc->usMiddleDutyCycle;
}

static Servo* ServoGetById (eDEVICE_ID_t deviceId) {

    if (DEVICE_ID_IS_SERVO (deviceId) == FALSE) {
        LOG_ERROR ("deviceId is not servo");
        return NULL;
    }

    return &gServos[SERVO_ID2IDX (deviceId)];
}

eSTATUS_t ServoInit (ServoInitConf_t conf) {

    Servo* pServo = ServoGetById (conf.id);
    if (pServo == NULL) {
        LOG_ERROR ("Failed to get Servo by ID");
        return eSTATUS_FAILURE;
    }
    memset (pServo, 0, sizeof (Servo));
    pServo->conf                   = conf;
    pServo->desc.usLeftDutyCycle   = 500;
    pServo->desc.usMiddleDutyCycle = 1500;
    pServo->desc.usRightDutyCycle  = 2500;
    pServo->desc.maxAngle          = 90.0F;
    pServo->desc.usableMaxAngle    = 25.0F;

    TimerInitConf_t timerConf =
    TIMER_CREATE_PWM_CONF (conf.id, conf.timerId, 50U, TRUE);
    if (TimerInit (timerConf) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize timer for Servo");
        goto error;
    }

    /*
     * NOTE: When given a duty cycle of 2500us the servo spins CW (POV is
     * servo shaft is pointed to the ceiling and its being viewed from
     * above). For example, if the servo is mounted on the left side of the
     * drone (POV directly behind drone), pid will respond to a positive
     * roll with a negative pid roll. The left servo should respond to the
     * negative pid roll by spinning CW (positive angle, duty cycle >
     * 1600us) so the sign of servo pitch mix needs to be negative. This
     * flips the sign of the pid pitch from negative to positive so the
     * left servo spins CW.
     */
    eDEVICE_ID_t id = conf.id;
    if (id == eLEFT_SERVO_MOTOR_ID) {
        pServo->desc.rollMix = LEFT_SERVO_PID_ROLL_MIX_DIR * SERVO_PID_ROLL_MIX;
        pServo->desc.yawMix = LEFT_SERVO_PID_YAW_MIX_DIR * SERVO_PID_YAW_MIX;
        pServo->desc.pitchMix = LEFT_SERVO_PID_PITCH_MIX_DIR * SERVO_PID_PITCH_MIX;
    } else if (id == eRIGHT_SERVO_MOTOR_ID) {
        pServo->desc.rollMix = RIGHT_SERVO_PID_ROLL_MIX_DIR * SERVO_PID_ROLL_MIX;
        pServo->desc.yawMix = RIGHT_SERVO_PID_YAW_MIX_DIR * SERVO_PID_YAW_MIX;
        pServo->desc.pitchMix = RIGHT_SERVO_PID_PITCH_MIX_DIR * SERVO_PID_PITCH_MIX;
    } else {
        LOG_ERROR ("Invalid actuator ID for Servo: %u", id);
        goto error;
    }

    return eSTATUS_SUCCESS;

error:
    memset (pServo, 0, sizeof (Servo));
    return eSTATUS_FAILURE;
}

eSTATUS_t ServoStart (eDEVICE_ID_t servoId) {

    Servo* pServo = ServoGetById (servoId);
    if (pServo == NULL) {
        LOG_ERROR ("Failed to get Servo by ID");
        return eSTATUS_FAILURE;
    }

    if (TimerStart (pServo->conf.timerId, NULL, 0) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start Timer for Servo");
        return eSTATUS_FAILURE;
    }

    // Set initial angle to middle position
    return ServoWrite (servoId, 0.0F);
}

eSTATUS_t ServoStop (eDEVICE_ID_t servoId) {

    Servo* pServo = ServoGetById (servoId);
    if (pServo == NULL) {
        LOG_ERROR ("Failed to get Servo by ID");
        return eSTATUS_FAILURE;
    }

    if (TimerStop (pServo->conf.timerId) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to stop Timer for Servo");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t ServoWrite (eDEVICE_ID_t servoId, float targetAngle) {

    Servo* pServo = ServoGetById (servoId);
    if (pServo == NULL) {
        LOG_ERROR ("Failed to get Servo by ID");
        return eSTATUS_FAILURE;
    }

    float clippedAngle =
    clipf32 (targetAngle, -pServo->desc.usableMaxAngle, pServo->desc.usableMaxAngle);
    pServo->curAngle       = clippedAngle;
    pServo->curTargetAngle = targetAngle;

    return TimerWrite (pServo->conf.timerId, (uint32_t)ServoAngle2PWM (pServo, clippedAngle));
}


#ifndef UNIT_TEST

#endif // UNIT_TEST

static Motor* MotorGetById (eDEVICE_ID_t deviceId) {

    if (DEVICE_ID_IS_MOTOR (deviceId) == FALSE) {
        return NULL;
    }

    return &gMotors[MOTOR_ID2IDX (deviceId)];
}

eSTATUS_t MotorInit (MotorInitConf_t conf, GPIO_TypeDef* pPort, uint16_t pin) {

    Motor* pMotor = MotorGetById (conf.id);
    if (pMotor == NULL) {
        LOG_ERROR ("Failed to get Motor by ID");
        return eSTATUS_FAILURE;
    }

    memset (pMotor, 0, sizeof (Motor));
    pMotor->conf = conf;

    DShotInitConf_t dshotConfig = { 0 };
    dshotConfig.deviceId        = conf.id;
    dshotConfig.timerId         = conf.timerId;
    dshotConfig.dshotType       = conf.dshotType;
    dshotConfig.gpio.pPort      = pPort;
    dshotConfig.gpio.pin        = pin;

    if (DShotInit (dshotConfig) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize DShot for Motor");
        goto error;
    }

    eDEVICE_ID_t id = conf.id;
    if (id == eLEFT_MOTOR_ID) {
        pMotor->desc.pitchMix = MOTOR_PID_PITCH_MIX * LEFT_MOTOR_PID_PITCH_MIX_DIR;
        pMotor->desc.yawMix = MOTOR_PID_YAW_MIX * LEFT_MOTOR_PID_YAW_MIX_DIR;
        pMotor->desc.rollMix = MOTOR_PID_ROLL_MIX * LEFT_MOTOR_PID_ROLL_MIX_DIR;

    } else if (id == eRIGHT_MOTOR_ID) {
        pMotor->desc.pitchMix = MOTOR_PID_PITCH_MIX * RIGHT_MOTOR_PID_PITCH_MIX_DIR;
        pMotor->desc.yawMix = MOTOR_PID_YAW_MIX * RIGHT_MOTOR_PID_YAW_MIX_DIR;
        pMotor->desc.rollMix = MOTOR_PID_ROLL_MIX * RIGHT_MOTOR_PID_ROLL_MIX_DIR;
    } else {
        LOG_ERROR ("Invalid actuator ID for Motor: %u", id);
        goto error;
    }

    return eSTATUS_SUCCESS;

error:
    memset (pMotor, 0, sizeof (Motor));
    return eSTATUS_FAILURE;
}

eSTATUS_t MotorStart (eDEVICE_ID_t motorId) {

    if (DEVICE_ID_IS_MOTOR (motorId) == FALSE) {
        LOG_ERROR ("deviceId is not motor");
        return eSTATUS_FAILURE;
    }

    if (DShotStart (motorId) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start DShot for Motor");
        return eSTATUS_FAILURE;
    }
    // Set initial throttle to minimum
    // return DShotWrite (&pMotor->dshot, DSHOT_MIN_THROTTLE);
    return eSTATUS_SUCCESS;
}

eSTATUS_t MotorStop (eDEVICE_ID_t motorId) {

    /*
     * NOTE: The motor's ESC will automatically stop the motor
     * when the PWM signal has stopped for more than 5-10ms.
     */
    if (DShotStop (motorId) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to stop dshot");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

/*
 * throttle is between 0.0F and 1.0F
 */
eSTATUS_t MotorWrite (eDEVICE_ID_t motorId, float targetThrottle) {

    Motor* pMotor = MotorGetById (motorId);
    if (pMotor == NULL) {
        LOG_ERROR ("Received NULL pointer for Motor");
        return eSTATUS_FAILURE;
    }

    if (targetThrottle < 0.0F || targetThrottle > 1.0F) {
        LOG_ERROR ("Motor throttle out of range: %u", (uint16_t)(targetThrottle * 100.0F));
        return eSTATUS_FAILURE;
    }

    float clippedThrottle =
    clipf32 (targetThrottle, MOTOR_MIN_THROTTLE, MOTOR_MAX_THROTTLE);
    pMotor->curThrottle       = clippedThrottle;
    pMotor->curTargetThrottle = targetThrottle;

    eSTATUS_t status =
    DShotWrite (motorId, DSHOT_MIN_THROTTLE + (uint16_t)(clippedThrottle * (float)DSHOT_RANGE));
    if (status != eSTATUS_SUCCESS && status != eSTATUS_BUSY) {
        LOG_ERROR ("Failed to write to motor");
        return status;
    }
    /*
     * NOTE: DShotWrite returns eSTATUS_BUSY when a write is in progress.
     * Don't consider this a failure.
     */
    return eSTATUS_SUCCESS;
}

eSTATUS_t MotorWriteCmd (eDEVICE_ID_t motorId, eMOTOR_CMD_t command) {


    if (DEVICE_ID_IS_MOTOR (motorId) == FALSE) {
        LOG_ERROR ("deviceId is not motor");
        return eSTATUS_FAILURE;
    }

    switch (command) {
    // NOTE: The arm and disarm command values are the same and depend on the ESC's current state.
    case eMOTOR_CMD_ARM: break;
    // case eMOTOR_CMD_DISARM: break;
    default: LOG_ERROR ("Unknown motor command"); return eSTATUS_FAILURE;
    }

    eSTATUS_t status = DShotWrite (motorId, command);
    if (status != eSTATUS_SUCCESS && status != eSTATUS_BUSY) {
        LOG_ERROR ("Failed to write command to motor");
        return status;
    }

    return eSTATUS_SUCCESS;
}

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static Motor gLeftMotor;
static Servo gLeftServo;
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

#ifndef UNIT_TEST

static eSTATUS_t
ActuatorsMixPair (eDEVICE_ID_t servoId, eDEVICE_ID_t motorId, Vec3f pidAttitude, float targetThrottle);
static eSTATUS_t ActuatorsArm (void);

#endif // UNIT_TEST

/*
 * \param pidAttitude roll, pitch, and yaw are between -1 and 1
 */
STATIC_TESTABLE_DECL eSTATUS_t
ActuatorsMixPair (eDEVICE_ID_t servoId, eDEVICE_ID_t motorId, Vec3f pidAttitude, float targetThrottle) {
    /*
     *  Motor Mixing
     */
    Motor* pMotor               = MotorGetById (motorId);
    MotorDescriptor* pMotorDesc = &pMotor->desc;
    float mixedThrottle         = targetThrottle; // between 0 and 1
    /*
     * NOTE: A PID pitch value should always increase the throttle of both
     * motors regardless of the sign of the PID pitch value.
     */
    mixedThrottle += pMotorDesc->pitchMix * ABS_F32 (pidAttitude.pitch) +
                     pMotorDesc->rollMix * pidAttitude.roll +
                     pMotorDesc->yawMix * pidAttitude.yaw;
    mixedThrottle = clipf32 (mixedThrottle, 0.0F, 1.0F);

    /*
     *  Servo Mixing
     */
    Servo* pServo               = ServoGetById (servoId);
    ServoDescriptor* pServoDesc = &pServo->desc;
    float mixedAngle = pServoDesc->pitchMix * pidAttitude.pitch +
                       pServoDesc->rollMix * pidAttitude.roll +
                       pServoDesc->yawMix * pidAttitude.yaw;
    // NOTE: Maybe Roll should have a negative impact on target angle.
    // Meaning the magnitude of the target angle is closer to 0 the larger
    // pid roll is.
    mixedAngle = clipf32 (mixedAngle, -1.0F, 1.0F) * pServoDesc->maxAngle;

    /* Motor throttle should be between 0 and 1 */
    MotorWrite (motorId, mixedThrottle);
    ServoWrite (servoId, mixedAngle);
    return eSTATUS_SUCCESS;
}

/*
 * \brief After motors are armed, a motor write has to be issued at
 * least every 5ms or the motor ESC will stop the motor.
 */
STATIC_TESTABLE_DECL eSTATUS_t ActuatorsArm (void) {

    // #ifndef USE_SERVOS_ONLY

    uint32_t msDelay    = 2;
    uint32_t msMaxTime  = 350;
    uint32_t iterations = msMaxTime / msDelay;
    for (uint32_t i = 0; i < iterations; ++i) {
        /* NOTE: A DShot value of all 0s is a special command to
         * the esc to arm/disarm the motor depending on the esc's current state.
         * The reason MotorWrite isn't used is because it uses a valid throttle value between > 48 and < 2048 */
        for (uint32_t motorIdx = 0; motorIdx < eMOTOR_ID_MAX; ++motorIdx) {
            Motor* pMotor = &gMotors[motorIdx];
            if (MotorWriteCmd (pMotor->conf.id, eMOTOR_CMD_ARM) != eSTATUS_SUCCESS) {
                LOG_ERROR ("Failed to arm motor ID %u", pMotor->conf.id);
                return eSTATUS_FAILURE;
            }
        }
        // NOTE: assumes DShot150 is used.
        vTaskDelay (pdMS_TO_TICKS (msDelay));
    }

    // Slowly increase the throttle to 15%
    msDelay              = 4;
    msMaxTime            = 3000;
    float i              = MOTOR_MIN_THROTTLE;
    float targetThrottle = MOTOR_STARTUP_THROTTLE;
    float increment      = targetThrottle / (float)(msMaxTime / msDelay);
    while (i < targetThrottle) {
        for (uint32_t motorIdx = 0; motorIdx < eMOTOR_ID_MAX; ++motorIdx) {
            Motor* pMotor = &gMotors[motorIdx];
            if (MotorWrite (pMotor->conf.id, i) != eSTATUS_SUCCESS) {
                LOG_ERROR ("Failed to arm motor ID %u", pMotor->conf.id);
                return eSTATUS_FAILURE;
            }
        }
        vTaskDelay (pdMS_TO_TICKS (msDelay));
        i += increment;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t ActuatorsInit () {

    eSTATUS_t status = ServoInit (eLEFT_SERVO_1_ID, left_ServoPWM, &gLeftServo);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize left servo");
        return status;
    }

    status = MotorInit (eLEFT_MOTOR_ID, left_Motor, &gLeftMotor);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize left motor");
        return status;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t ActuatorsStart (void) {

    eSTATUS_t status = ServoStart (&gLeftServo);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start left servo");
        return status;
    }

    status = MotorStart (&gLeftMotor);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start left motor");
        return status;
    }

    status = ActuatorsArm ();
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to arm actuators");
        return status;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t ActuatorsStop (void) {

    if (MotorStop (&gLeftMotor) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to stop left motor");
        return eSTATUS_FAILURE;
    }

    if (ServoStop (&gLeftServo) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to stop left servo");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t ActuatorsWrite (Vec3f pidAttitude, float targetThrottle) {
    return ActuatorsMixPair (&gLeftServo, &gLeftMotor, pidAttitude, targetThrottle);
}

Servo* ActuatorsGetLeftServo (void) {
    return &gLeftServo;
}

void ActuatorsLogData (void) {
    LOG_DATA_ACTUATORS_DATA ("Left Motor", gLeftMotor, "Left Servo", gLeftServo);
}