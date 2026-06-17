#ifndef CONTROL_RC_H
#define CONTROL_RC_H

#include "core/core.h"

eSTATUS_t Rc_Init(void);
eSTATUS_t Rc_Update(void);

void Rc_Task(void* args);

#endif // CONTROL_RC_H
