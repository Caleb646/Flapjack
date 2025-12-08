#include <stdbool.h>
#include <stdint.h>

#include "cfg/cfg.h"
#include "cfg/motor.h"

#include "targets/target.h"

CFG_DEFINE_ARRAY (MotorCfg_t, MotorCfgs, TARG_MAX_MOTORS);
CFG_DEFINE_ARRAY (ServoCfg_t, ServoCfgs, TARG_MAX_SERVOS);