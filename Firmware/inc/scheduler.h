#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

#include "fj_task.h"

void Scheduler_Main (uint32_t coreIdx, uint32_t loopRateHz);


#endif /* SCHEDULER_H */