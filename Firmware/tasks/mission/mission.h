#ifndef MISSION_MISSION_H
#define MISSION_MISSION_H

#include "core/core.h"

eSTATUS_t Mission_Init(void);
eSTATUS_t Mission_Update(void);

void Mission_Task(void* args);

#endif // MISSION_MISSION_H
