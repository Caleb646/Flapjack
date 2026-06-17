#ifndef GUIDANCE_GUIDANCE_H
#define GUIDANCE_GUIDANCE_H

#include "core/core.h"

eSTATUS_t Guidance_Init(void);
eSTATUS_t Guidance_Update(void);

void Guidance_Task(void* args);

#endif // GUIDANCE_GUIDANCE_H
