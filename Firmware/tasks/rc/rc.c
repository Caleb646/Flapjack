#include "core/core.h"

#include "tasks/rc/rc.h"

#include "drivers/rx/rx.h"

#include "umsg_rc.h"

#include "FreeRTOS.h"
#include "task.h"

/*
 * The RC sensor layer: poll the receiver, publish what it is saying.
 *
 * Same shape as every other sensor task - Imu_Task / Mag_Task each own their
 * device and publish one topic. The receiver protocol itself stays behind
 * drivers/rx (rx.c + crsf.c), which is what makes swapping CRSF for another
 * receiver a driver change rather than a task change.
 *
 * This used to be two tasks with a umsg topic between them. That split was a
 * core-allocation artifact - the decode half ran on CM4 and published across
 * shared memory to CM7 - and once both moved to CM7 it only bought a second
 * 20 ms polling hop, doubling stick-to-guidance latency for no benefit.
 */

typedef struct {
    uint32_t usLastUpdate;
} Rc_t;

static Rc_t s_Rc;

eSTATUS_t Rc_Init(void) {
    s_Rc.usLastUpdate = GetMicroseconds();
    return Rx_Init();
}

eSTATUS_t Rc_Update(void) {
    uint32_t const usNow = GetMicroseconds();

    /* A failure here is the ordinary case, not an error: the receiver is polled
     * faster than whole frames arrive, so most calls find nothing new. Publish
     * regardless - holding the last sticks and reporting the link state is what
     * lets a lost link be seen downstream instead of the chain simply going
     * quiet. */
    (void)Rx_Update(usNow, usNow - s_Rc.usLastUpdate);
    s_Rc.usLastUpdate = usNow;

    uint32_t const* ch = Rx_GetChannels();

    // rssi stays 0 until the CRSF 0x14 Link Statistics frame is decoded;
    // link_quality is all-or-nothing for now, from the receiver's frame timeout.
    umsg_rc_input_t msg = { .rssi = 0, .link_quality = Rx_IsLinkUp() ? 100U : 0U };
    for (uint32_t i = 0; i < RC_MAX_CHANNELS; i++) {
        msg.channels[i] = ch[i];
    }
    umsg_rc_input_publish(&msg);
    return eSTATUS_SUCCESS;
}

void Rc_Task(void* args) {
    (void)args;
    if (STATUS_FAIL(Rc_Init())) {
        LOG_ERROR("RC unavailable; task exiting");
        /* Not vTaskDelete: this build uses heap_1, whose vPortFree asserts -
         * and configASSERT spins with interrupts disabled, wedging the FC. */
        vTaskSuspend(NULL);
        return;
    }
    while (1) {
        Rc_Update();
        /* Rc_Update() never blocks - the receiver is fed by a UART ISR, so there
         * is nothing to wait on - and without this the task stays permanently
         * ready and starves every task below TASK_PRIORITY_SENSOR_RC. */
        vTaskDelay(pdMS_TO_TICKS(20));   // 50 Hz
    }
}
