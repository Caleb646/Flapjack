#ifndef NAV_NAV_H
#define NAV_NAV_H

#include "core/core.h"

eSTATUS_t Nav_Init(void);
eSTATUS_t Nav_Update(void);

void Nav_Task(void* args);

#endif // NAV_NAV_H
