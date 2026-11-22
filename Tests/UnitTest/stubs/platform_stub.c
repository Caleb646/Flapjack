#include "platform.h"

/* Time does not advance on the host: anything gated on elapsed time is
 * unreachable from a unit test. */
uint32_t Platform_GetMicroseconds (void) {
    return 0;
}

bool Platform_TakePowerOnReset (void) {
    return true;
}

uint32_t Platform_CurrentException (void) {
    return 0;
}

void Platform_Reset (void) {
    while (1) {
    }
}

void Platform_NotifyOtherCore (void) {
}

bool Platform_LockTake (void) {
    return true;
}

void Platform_LockRelease (void) {
}
