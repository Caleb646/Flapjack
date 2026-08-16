#ifndef SHELL_H
#define SHELL_H

#include "core/core.h"

/*
 * Registers the command handler with SerialLink; there is no shell task. Frames
 * are decoded in the SerialLink RX task, so nothing needs to poll.
 */
eSTATUS_t Shell_Init (void);

#endif // SHELL_H
