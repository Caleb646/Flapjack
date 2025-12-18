#ifndef AERO_PID_H
#define AERO_PID_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "cfg/cfg.h"

typedef struct PidAxisCfg_s {
    float p;
    float i;
    float d;
    float integralLimit;
} PidAxisCfg_t;

typedef struct PidCfg_s {
    PidAxisCfg_t axis[AXIS_IDX_COUNT];
} PidCfg_t;

typedef struct PidAxisData_s {
    float attitude;
    float error;
    float integral;
    float derivative;
    float prevError;
    float prevIntegral;
} PidAxisData_t;

typedef struct PidData_s {
    PidAxisData_t axis[AXIS_IDX_COUNT];
    float pidAttitude[AXIS_IDX_COUNT];
    uint32_t usLastUpdate;
} PidData_t;

CFG_DECLARE (PidCfg_t, PidCfg);
FJ_DECLARE_SHARED (PidData_t, e_PidData);

eSTATUS_t PidData_Init (void);
eSTATUS_t Pid_Update (uint32_t usCurrentTime);

#endif // AERO_PID_H