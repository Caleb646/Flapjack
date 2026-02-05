#ifndef DRIVERS_MOTION_H
#define DRIVERS_MOTION_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "cfg/cfg.h"

#include "drivers/io/gpio_defs.h"

typedef uint8_t eMOTOR_PROT_t;
enum {
    eMOTION_PROT_TYPE_NULL = 0,

    I_eMOTION_PROT_TYPE_DSHOT,
    eMOTION_PROT_TYPE_DSHOT_150,

    eMOTION_PROT_TYPE_PWM,
};

#define MOTOR_PROT_IS_DSHOT(PROT_TYPE) ((PROT_TYPE) == eMOTION_PROT_TYPE_DSHOT_150)
#define MOTOR_PROT_MAKE_EXPAND(PROT_NAME) eMOTION_PROT_TYPE_##PROT_NAME
#define MOTOR_PROT_MAKE(...)              MOTOR_PROT_MAKE_EXPAND (__VA_ARGS__)

typedef uint8_t eMOTOR_ID_t;
enum {
    eMOTOR_ID_NULL = 0,

    eMOTOR_1_ID,
    eMOTOR_LEFT_ID = eMOTOR_1_ID,

    eMOTOR_2_ID,
    eMOTOR_RIGHT_ID = eMOTOR_2_ID,

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
    eSERVO_ID_NONE = 0,

    eSERVO_1_ID,
    eSERVO_LEFT_MOTOR_ID = eSERVO_1_ID,

    eSERVO_2_ID,
    eSERVO_RIGHT_MOTOR_ID = eSERVO_2_ID,

    eSERVO_3_ID,
    eSERVO_LEFT_AILERON_ID = eSERVO_3_ID,

    eSERVO_4_ID,
    eSERVO_RIGHT_AILERON_ID = eSERVO_4_ID,

    eSERVO_5_ID,
    eSERVO_RUDDER_ID = eSERVO_5_ID,

    eSERVO_6_ID,
    eSERVO_ELEVATOR_ID = eSERVO_6_ID,

    eSERVO_7_ID,
    eSERVO_8_ID
};

#define SERVO_ID_TO_INDEX(SERVO_ID) ((SERVO_ID) - 1U)
#define SERVO_DEF_PERIOD_HZ         50U
#define SERVO_DEF_DC_LEFT_US        1000U
#define SERVO_DEF_DC_MIDDLE_US      1500U
#define SERVO_DEF_DC_RIGHT_US       2000U

typedef struct MotorsCfg_s {
    eGPIO_ID_t gpios[TARG_MAX_MOTORS];
    eMOTOR_PROT_t protType;
    uint8_t nMotors;
} MotorsCfg_t;

typedef struct ServosCfg_s {
    eGPIO_ID_t gpios[TARG_MAX_SERVOS];
    uint8_t nServos;
} ServosCfg_t;

typedef struct MotorsDevice_s {
    eSTATUS_t (*fnUpdateMotors) (float const* pThrottles, uint32_t nThrottles);
} MotorsDevice_t;

FJ_DECLARE_SHARED (MotorsCfg_t, e_MotorsCfg);
FJ_DECLARE_SHARED (ServosCfg_t, e_ServosCfg);
FJ_DECLARE_SHARED (TimChannel_t, e_TimChans[TARG_MAX_SERVOS]);
FJ_DECLARE_SHARED (MotorsDevice_t, e_MotorsDevice);

eSTATUS_t Motors_Init (void);
eSTATUS_t Motors_Update (float const* pThrottles, uint32_t nThrottles);

eSTATUS_t Servos_Init (void);
eSTATUS_t Servos_Update (uint16_t const* pOutputs, uint32_t nOutputs);

#endif // DRIVERS_MOTION_H