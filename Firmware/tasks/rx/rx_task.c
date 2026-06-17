#include "tasks/rx/rx_task.h"

#include "core/core.h"

#include "drivers/rx/rx.h"

void Rx_Task(void* args) {
    (void)args;
    Rx_Init();
    uint32_t usLast = GetMicroseconds();
    while (1) {
        uint32_t usNow = GetMicroseconds();
        Rx_Update(usNow, usNow - usLast);
        usLast = usNow;
    }
}
