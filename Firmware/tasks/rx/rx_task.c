#include "tasks/rx/rx_task.h"

#include "core/core.h"

#include "drivers/rx/rx.h"

#include "FreeRTOS.h"
#include "task.h"

void Rx_Task(void* args) {
    (void)args;
    Rx_Init();
    uint32_t usLast = GetMicroseconds();
    while (1) {
        uint32_t usNow = GetMicroseconds();
        Rx_Update(usNow, usNow - usLast);
        usLast = usNow;
        /* Rx_Update() never blocks, so without this the task stays permanently
         * ready and starves every task below TASK_PRIORITY_RX. Harmless while
         * this only ran on CM4, which has nothing else to starve; it stopped
         * being harmless as soon as SINGLE_CORE put it on CM7 beside the GNC
         * loop. Same fault and same fix as Rc_Task - see KnownIssues 2.4. */
        vTaskDelay(pdMS_TO_TICKS(20));   // 50 Hz, matching Rc_Task
    }
}
