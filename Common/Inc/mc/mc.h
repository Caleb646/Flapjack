
#ifndef MC_MC_H
#define MC_MC_H

#include "core/core.h"
#include "mc/actuators.h"
#include "mc/dshot.h"
#include "mc/filter.h"
#include "mc/pid.h"

eSTATUS_t MC_InitAll (void);
eSTATUS_t MC_StartAll (void);

#endif /* MC_MC_H */