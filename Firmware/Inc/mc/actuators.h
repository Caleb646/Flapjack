#ifndef MOTION_CONTROL_ACTUATORS_H
#define MOTION_CONTROL_ACTUATORS_H

#include "conf/board.h"
#include "conf/conf.h"
#include "core/core.h"
#include "device/motor/motor.h"
#include "device/servo/servo.h"
#include "hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    DevDesc_t* pDevDesc;
} ActuatorInitConf_t;

typedef struct {
    eDEVICE_ID_t actId;
    eDEVICE_ID_t linkedActId;
    union {
        Motor_t* pLinkedMotor;
        Servo_t* pLinkedServo;
    };
    PIDDesc_t pidDesc;
    ActProtDesc_t actProtDesc;
    union {
        Motor_t motor;
        Servo_t servo;
    };
    bool isInitialized;
} Actuator_t;

typedef struct {
    Actuator_t pMotors[MOTOR_MAX_MOTORS];
    Actuator_t pServos[SERVO_MAX_SERVOS];
    uint32_t motorCount;
    uint32_t servoCount;
} ActuatorSystem_t;


#ifdef UNIT_TEST
eSTATUS_t ActuatorsMixPair (Servo_t* pServo, Motor_t* pMotor, Vec3f pidAttitude, float targetThrottle);
eSTATUS_t ActuatorsArm (Motor_t* pMotor);
#endif // UNIT_TEST

eSTATUS_t Actuator_Init (ActuatorInitConf_t conf, Actuator_t* pOutActuator);

eSTATUS_t ActuatorsStart (void);
eSTATUS_t ActuatorsStop (void);
eSTATUS_t Actuators_Update (Vec3f pidAttitude, float targetThrottle);
void ActuatorsLogData (void);

#endif // MOTION_CONTROL_ACTUATORS_H
