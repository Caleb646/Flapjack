
#ifndef MC_MC_H
#define MC_MC_H

#include "common.h"
#include "mc/actuators.h"
#include "mc/dshot.h"
#include "mc/filter.h"
#include "mc/pid.h"

eSTATUS_t MCInitAll (void);
eSTATUS_t MCStartAll (void);

#endif /* MC_MC_H */