#ifndef CFG_MOTION_H
#define CFG_MOTION_H

#include <stdbool.h>
#include <stdint.h>

#include "drivers/motor.h"

static inline void Cfg_Motor_Init (eMOTOR_ID_t motorId, eGPIO_ID_t gpioId) {
    MotorsCfg_GetMutable ()->gpios[MOTOR_ID_TO_INDEX (motorId)] = gpioId;
    MotorsCfg_GetMutable ()->nMotors++;
}

#define CFG_MOTOR_INIT(MOTOR_ID, GPIO_ID)                                       \
    do {                                                                        \
        MotorsCfg_GetMutable ()->gpios[MOTOR_ID_TO_INDEX (MOTOR_ID)] = GPIO_ID; \
        MotorsCfg_GetMutable ()->nMotors++;                                     \
    } while (0)

#define CFG_MOTOR_SET_MIX(MOTOR_ID, PITCH_MIX, ROLL_MIX, YAW_MIX)                       \
    do {                                                                                \
        MotorsCfg_GetMutable ()->mixes[MOTOR_ID_TO_INDEX (MOTOR_ID)].pitch = PITCH_MIX; \
        MotorsCfg_GetMutable ()->mixes[MOTOR_ID_TO_INDEX (MOTOR_ID)].roll  = ROLL_MIX;  \
        MotorsCfg_GetMutable ()->mixes[MOTOR_ID_TO_INDEX (MOTOR_ID)].yaw   = YAW_MIX;   \
    } while (0)

#define CFG_MOTOR_LINK_SERVO(MOTOR_ID, SERVO_ID)                                           \
    do {                                                                                   \
        MotorsCfg_GetMutable ()->linkedServoIds[MOTOR_ID_TO_INDEX (MOTOR_ID)]  = SERVO_ID; \
        ServosCfg_GetMutable ()->linkedMotorsIds[SERVO_ID_TO_INDEX (SERVO_ID)] = MOTOR_ID; \
    } while (0)

#endif // CFG_MOTION_H