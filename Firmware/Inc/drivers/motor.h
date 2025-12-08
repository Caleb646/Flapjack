#ifndef DRIVERS_MOTION_H
#define DRIVERS_MOTION_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "drivers/driver.h"

#include "drivers/io/gpio_defs.h"

#include "drivers/tim_defs.h"

typedef uint8_t eMOTION_PROT_TYPE_t;
enum {
    eMOTION_PROT_TYPE_NULL = 0,
    eMOTION_PROT_TYPE_DSHOT,
    eMOTION_PROT_TYPE_PWM,
};

typedef uint8_t eMOTOR_ID_t;
enum {
    eMOTOR_ID_NULL = 0,
    eMOTOR_1_ID,
    eMOTOR_2_ID,
    eMOTOR_3_ID,
    eMOTOR_4_ID,
    eMOTOR_5_ID,
    eMOTOR_6_ID,
    eMOTOR_7_ID,
    eMOTOR_8_ID
};

#define MOTOR_ID_TO_INDEX(MOTOR_ID) ((MOTOR_ID) - 1U)

typedef uint8_t eSERVO_ID_t;
enum {
    eSERVO_ID_NULL = 0,
    eSERVO_1_ID,
    eSERVO_2_ID,
    eSERVO_3_ID,
    eSERVO_4_ID,
    eSERVO_5_ID,
    eSERVO_6_ID,
    eSERVO_7_ID,
    eSERVO_8_ID
};

#define SERVO_ID_TO_INDEX(SERVO_ID) ((SERVO_ID) - 1U)

typedef struct ServoDevice_s ServoDevice_t;

typedef struct MotorDevice_s {
    eMOTOR_ID_t id;
    eMOTION_PROT_TYPE_t protType;
    ServoDevice_t* pLinkedServoDev;
    bool isDeviceInitialized;
} MotorDevice_t;

typedef struct ServoDevice_s {
    eSERVO_ID_t id;
    eMOTION_PROT_TYPE_t protType;
    MotorDevice_t* pLinkedMotorDev;
    bool isDeviceInitialized;
} ServoDevice_t;

typedef struct MotionControl_s {
    MotorDevice_t motors[TARG_MAX_MOTORS];
    ServoDevice_t servos[TARG_MAX_SERVOS];
    void (*fnMixerUpdate) (float dt);
    uint8_t nMotors;
    uint8_t nServos;
} MotionControl_t;


DRIVER_DECLARE_ARRAY (MotorDevice_t, MotorDevices, TARG_MAX_MOTORS);
DRIVER_DECLARE_ARRAY (ServoDevice_t, ServoDevices, TARG_MAX_SERVOS);

typedef struct MotorCfg_s MotorCfg_t;
uint8_t Motor_GetNumMotors (void);
eSTATUS_t Motor_Init (MotorCfg_t* pMotorCfg, MotorDevice_t* pOutMotor);

typedef struct ServoCfg_s ServoCfg_t;
uint8_t Servo_GetNumServos (void);
eSTATUS_t Servo_Init (ServoCfg_t* pServoCfg, ServoDevice_t* pOutServo);

#endif // DRIVERS_MOTION_H