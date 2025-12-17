#ifndef AERO_PID_H
#define AERO_PID_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

typedef struct PidCfg_s {
    Vec3f p;
    Vec3f i;
    Vec3f d;
} PidCfg_t;

eSTATUS_t PID_Init (void);
eSTATUS_t PID_Update (float dt);


#endif // AERO_PID_H