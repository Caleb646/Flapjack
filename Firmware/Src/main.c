#include "common.h"
#include "conf/conf.h"
#include "core/core.h"
#include "device/device.h"
#include "hal.h"
#include "mc/mc.h"
#include "scheduler.h"

int main (void) {

    if (FJ_FAIL (Core_Init ())) {
        CriticalErrorHandler ();
    }

    if (IS_CM7_ME ()) {

        Delay (100); // Wait for CM4 to initialize logger

        if (FJ_FAIL (Device_InitAll (BoardConfGet ()))) {
            CriticalErrorHandler ();
        }

        LOG_INFO ("Initializing Motion Control Module");
        if (FJ_FAIL (MC_InitAll ())) {
            LOG_ERROR ("Failed to init motion control module");
            CriticalErrorHandler ();
        }

    } else {
        // LOG_INFO ("CM4 core initialized");
    }

    LOG_INFO ("Core Initialized. Starting scheduler");
    Scheduler_Start ();
    return 0;
}