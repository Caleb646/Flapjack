#include "core/core.h"

#include "tasks/rc/rc.h"

#include "drivers/rx/rx.h"

#include "umsg_rc.h"

#include "FreeRTOS.h"
#include "task.h"

eSTATUS_t Rc_Init(void) {
    return eSTATUS_SUCCESS;
}

eSTATUS_t Rc_Update(void) {
    uint32_t const* ch = Rx_GetChannels();
    if (!ch) {
        return eSTATUS_FAILURE;
    }

    /* umsg_publish() stores a pointer to this buffer rather than copying it, so
     * it must outlive Rc_Update() - otherwise umsg_rc_input_peek() (guidance)
     * reads this frame after vTaskDelay() has reused the stack. */
    static umsg_rc_input_t msg = { .rssi = 0, .link_quality = 0 };
    for (uint8_t i = 0; i < RC_MAX_CHANNELS; i++) {
        msg.channels[i] = ch[i];
    }
    umsg_rc_input_publish(&msg);
    return eSTATUS_SUCCESS;
}

void Rc_Task(void* args) {
    (void)args;
    Rc_Init();
    while (1) {
        Rc_Update();
        /* Rc_Update() never blocks, so without this the task stays permanently
         * ready and starves every task below TASK_PRIORITY_SENSOR_RC. */
        vTaskDelay(pdMS_TO_TICKS(20));   // 50 Hz
    }
}
