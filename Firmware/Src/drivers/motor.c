#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "core/core.h"

#include "drivers/tim.h"
#include "drivers/tim_defs.h"

#include "drivers/motor.h"

#include "cfg/motor.h"

#include "targets/target.h"

static uint8_t g_NumMotors TARG_SHARED_MEM_SECTION = 0;
static uint8_t g_NumServos TARG_SHARED_MEM_SECTION = 0;

DRIVER_DEFINE_ARRAY (MotorDevice_t, MotorDevices, TARG_MAX_MOTORS);
DRIVER_DEFINE_ARRAY (ServoDevice_t, ServoDevices, TARG_MAX_SERVOS);

uint8_t Motor_GetNumMotors (void) {
    return g_NumMotors;
}

eSTATUS_t Motor_Init (MotorCfg_t* pMotorCfg, MotorDevice_t* pOutMotor) {

    g_NumMotors++;
    return eSTATUS_SUCCESS;
}


uint8_t Servo_GetNumServos (void) {
    return g_NumServos;
}

eSTATUS_t Servo_Init (ServoCfg_t* pServoCfg, ServoDevice_t* pOutServo) {

    g_NumServos++;
    return eSTATUS_SUCCESS;
}