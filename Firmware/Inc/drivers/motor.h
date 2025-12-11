#ifndef DRIVERS_MOTION_H
#define DRIVERS_MOTION_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

typedef uint8_t eMOTOR_PROT_t;
enum {
    eMOTION_PROT_TYPE_NULL = 0,

    I_eMOTION_PROT_TYPE_DSHOT,
    eMOTION_PROT_TYPE_DSHOT_150,

    eMOTION_PROT_TYPE_PWM,
};

#define MOTOR_PROT_IS_DSHOT(PROT_TYPE) ((PROT_TYPE) == eMOTION_PROT_TYPE_DSHOT_150)

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

typedef struct Dshot_s Dshot_t;
typedef struct Motors_s Motors_t;
typedef struct Servos_s Servos_t;

typedef struct MotorProtVtbl_s {
    union {
        eSTATUS_t (*fnUpdateMotors) (float const* pThrottles, uint32_t nThrottles);
        eSTATUS_t (*fnUpdateServos) (float const* pAngles, uint32_t nAngles);
    };
} MotorProtVtbl_t;

eSTATUS_t Motors_Init (void);
eSTATUS_t Servos_Init (void);

#endif // DRIVERS_MOTION_H