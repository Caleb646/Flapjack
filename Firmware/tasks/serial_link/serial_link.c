#include "tasks/serial_link/serial_link.h"

#include "drivers/serial/serial_link.h"

void SerialLink_RxTask (void* args) {

    (void)args;
    for (;;) {
        SerialLink_RxUpdate ();   // blocks on the next inbound byte
    }
}

/*
 * Runs at the lowest application priority on purpose. A sustained log flood
 * keeps this task permanently runnable, and any higher priority would starve
 * mag (1) and simtlm (1) - exactly the fault recorded as KnownIssues 2.6. At
 * equal priority FreeRTOS time-slices instead (configUSE_TIME_SLICING defaults
 * to 1), so they keep running and the link degrades by dropping text.
 */
void SerialLink_TxTask (void* args) {

    (void)args;
    for (;;) {
        SerialLink_TxUpdate ();   // blocks until there is text to write
    }
}
