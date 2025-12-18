#include <stdbool.h>
#include <stdint.h>

#include "common.h"
#include "task.h"

#include "aero/flight.h"
#include "aero/mixer.h"
#include "aero/pid.h"

#include "drivers/sensors/sensor.h"

eSTATUS_t Task_UpdateAcc (Task_t* pSelf, uint32_t currentTimeUs) {

    FJ_UNUSED (pSelf);
    FJ_UNUSED (currentTimeUs);
    return Acc_Update (false);
}

eSTATUS_t Task_UpdateGyro (Task_t* pSelf, uint32_t currentTimeUs) {

    FJ_UNUSED (pSelf);
    FJ_UNUSED (currentTimeUs);
    return Gyro_Update (false);
}

eSTATUS_t Task_UpdateMag (Task_t* pSelf, uint32_t currentTimeUs) {

    FJ_UNUSED (pSelf);
    FJ_UNUSED (currentTimeUs);
    return Mag_Update (false);
}

eSTATUS_t Task_UpdateAttitude (Task_t* pSelf, uint32_t currentTimeUs) {

    FJ_UNUSED (pSelf);
    return Attitude_Update (currentTimeUs);
}

eSTATUS_t Task_UpdateAltitude (Task_t* pSelf, uint32_t currentTimeUs) {

    FJ_UNUSED (pSelf);
    return Altitude_Update (currentTimeUs);
}

eSTATUS_t Task_UpdatePid (Task_t* pSelf, uint32_t currentTimeUs) {

    FJ_UNUSED (pSelf);
    FJ_UNUSED (currentTimeUs);
    return Pid_Update (currentTimeUs);
}

eSTATUS_t Task_UpdateMixer (Task_t* pSelf, uint32_t currentTimeUs) {

    FJ_UNUSED (pSelf);
    FJ_UNUSED (currentTimeUs);
    return Mixer_Update ();
}