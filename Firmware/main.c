#include <stdio.h>
#include <string.h>

#include "hal.h"
#include "target.h"
#include "platform.h"

#include "core/core.h"

#include "mc/filter.h"

#include "shell/shell.h"

#include "drivers/dma.h"
#include "drivers/io/gpio.h"

#include "drivers/rx/rx.h"

#include "drivers/sensors/mag/mag.h"

#include "device/imu.h"
#include "device/serial.h"

FJ_DEFINE_SHARED (bool volatile, s_IsCM4Stuck)     = false;
FJ_DEFINE_SHARED (bool volatile, s_IsSystemInited) = false;
FJ_DEFINE_SHARED (bool volatile, s_IsCM4Ready)     = false;

int main (void) {

    if(Platform_Init() != 0) {
        CriticalErrorHandler ();
    }

#if defined(CORE_CM7)

    if (STATUS_FAIL (Spi_InitSystem ())) {
        CriticalErrorHandler ();
    }

    if (STATUS_FAIL (Uart_InitSystem ())) {
        CriticalErrorHandler ();
    }

    if (STATUS_FAIL (Core_Init ())) {
        CriticalErrorHandler ();
    }
    
    if (STATUS_FAIL (SerialDebug_Init ())) {
        CriticalErrorHandler ();
    }

    s_IsSystemInited = true;
    while (!s_IsCM4Ready) {
        // allow cm4 to initialize logger
    };

    if (STATUS_FAIL (Shell_Init ())) {
        LOG_ERROR ("Failed to init shell");
        CriticalErrorHandler ();
    }

    LOG_INFO ("Starting scheduler");
    FjTasks_Start (CM7_IDX);
#endif /* CORE_CM7 */

/*
* ******************* CM4 Start *******************
*/
#if defined(CORE_CM4)

    while (!s_IsSystemInited) {
        // wait for cm7 to init system
    };

    if (Core_Init () != eSTATUS_SUCCESS) {
        s_IsCM4Stuck = true;
        CriticalErrorHandler ();
    }

    s_IsCM4Ready = true;
    LOG_INFO ("Starting scheduler");
    FjTasks_Start (CM4_IDX);

#endif /* CORE_CM4 */

    while (1);
}