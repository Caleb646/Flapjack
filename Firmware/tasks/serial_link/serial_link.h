#ifndef TASKS_SERIAL_LINK_SERIAL_LINK_TASK_H
#define TASKS_SERIAL_LINK_SERIAL_LINK_TASK_H

/*
 * Task bodies for the shared serial link. The wire protocol, the UART and the
 * buffering all live in drivers/serial/serial_link.c; these are just the loops
 * that drive it, following the same shape as every other task here - the
 * _Update blocks internally, so the body needs no delay of its own.
 */

#include "core/core.h"

// Deframes inbound traffic and dispatches to the registered handlers.
void SerialLink_RxTask (void* args);

// Writes out buffered log text. Never logs - see serial_link.c.
void SerialLink_TxTask (void* args);

#endif // TASKS_SERIAL_LINK_SERIAL_LINK_TASK_H
