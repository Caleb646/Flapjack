#include "FreeRTOS.h"
#include "hal.h"
#include "task.h"

extern void xPortSysTickHandler (void);

#if (USE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION == 0)
void SysTick_Handler (void) {
    /* Clear overflow flag */
    SysTick->CTRL;

    if (xTaskGetSchedulerState () != taskSCHEDULER_NOT_STARTED) {
        /* Call tick handler */
        xPortSysTickHandler ();
    }
}
#endif