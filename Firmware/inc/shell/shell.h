#ifndef SHELL_H
#define SHELL_H

#include "core/core.h"

eSTATUS_t Shell_Init (void);
eSTATUS_t Shell_Update (uint32_t usCurrentTime, uint32_t usDeltaTime);

#endif // SHELL_H
