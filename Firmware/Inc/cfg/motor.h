#ifndef CFG_MOTION_H
#define CFG_MOTION_H

#include <stdbool.h>
#include <stdint.h>

#include "targets/target.h"

#include "cfg/cfg.h"

#include "drivers/io/gpio_defs.h"

#include "drivers/tim_defs.h"

#include "drivers/motor.h"

typedef struct MotorsCfg_s {
    eGPIO_ID_t gpios[TARG_MAX_MOTORS];
    Vec3f mixes[TARG_MAX_MOTORS];
    eMOTOR_PROT_t protType;
    eSERVO_ID_t linkedServoIds[TARG_MAX_MOTORS];
    uint8_t nMotors;
} MotorsCfg_t;

typedef struct ServosCfg_s {
    eGPIO_ID_t gpios[TARG_MAX_SERVOS];
    Vec3f mixes[TARG_MAX_SERVOS];
    eMOTOR_PROT_t protType;
    eMOTOR_ID_t linkedMotorIds[TARG_MAX_SERVOS];
    float maxAngleDeg;
    uint8_t nServos;
} ServosCfg_t;

CFG_DECLARE (MotorsCfg_t, MotorsCfg);
CFG_DECLARE (ServosCfg_t, ServosCfg);

#endif // CFG_MOTION_H