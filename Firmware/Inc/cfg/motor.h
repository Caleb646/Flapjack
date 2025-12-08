#ifndef CFG_MOTION_H
#define CFG_MOTION_H

#include <stdbool.h>
#include <stdint.h>

#include "targets/target.h"

#include "cfg/cfg.h"

#include "drivers/io/gpio_defs.h"

#include "drivers/tim_defs.h"

#include "drivers/motor.h"

typedef struct MotorCfg_s {
    eMOTOR_ID_t id;
    eMOTION_PROT_TYPE_t protType;
    eSERVO_ID_t linkedServoId;

    eGPIO_ID_t gpio;
    TimCfg_t timCfg;

    // float maxRPM;
    // float polePairs;
    // bool sensorless;
} MotorCfg_t;

typedef struct ServoCfg_s {
    eSERVO_ID_t id;
    eMOTION_PROT_TYPE_t protType;
    eMOTOR_ID_t linkedMotorId;

    eGPIO_ID_t gpio;
    TimCfg_t timCfg;
    // float maxRPM;
    // float polePairs;
    // bool sensorless;
} ServoCfg_t;

CFG_DECLARE_ARRAY (MotorCfg_t, MotorCfgs, TARG_MAX_MOTORS);
CFG_DECLARE_ARRAY (ServoCfg_t, ServoCfgs, TARG_MAX_SERVOS);

#endif // CFG_MOTION_H