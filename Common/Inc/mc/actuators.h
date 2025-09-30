#ifndef MOTION_CONTROL_ACTUATORS_H
#define MOTION_CONTROL_ACTUATORS_H

#include "common.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "device/motor/motor.h"
#include "device/servo/servo.h"
#include "hal.h"
#include "mc/dshot.h"
#include "peripheral/dma.h"
#include "peripheral/timer.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef UNIT_TEST
eSTATUS_t ActuatorsMixPair (Servo_t* pServo, Motor_t* pMotor, Vec3f pidAttitude, float targetThrottle);
eSTATUS_t ActuatorsArm (Motor_t* pMotor);
#endif // UNIT_TEST

eSTATUS_t ActuatorsStart (void);
eSTATUS_t ActuatorsStop (void);
eSTATUS_t ActuatorsWrite (Vec3f pidAttitude, float targetThrottle);
void ActuatorsLogData (void);

#endif // MOTION_CONTROL_ACTUATORS_H
