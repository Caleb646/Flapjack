#include <stdbool.h>
#include <stdint.h>

#include "common.h"
#include "fj_tasks.h"

#include "aero/flight.h"
#include "aero/mixer.h"
#include "aero/pid.h"

#include "drivers/sensors/sensor.h"

#include "platform/platform.h"
#include "targets/target.h"

FJ_DEFINE_SHARED (Task_t, e_PrimaryTasks[]) = {
    { .fn = Task_UpdateMain, .pArg = NULL, .pName = "Main Update" },
    { .fn = Task_UpdateAcc, .pArg = NULL, .pName = "Acc Update" },
    { .fn = Task_UpdateGyro, .pArg = NULL, .pName = "Gyro Update" },

#if TARG_MAG_ENABLED() && !TARG_DUAL_CORE_ENABLED()
    { .fn = Task_UpdateMag, .pArg = NULL, .pName = "Mag Update" },
#endif

#if TARG_BARO_ENABLED() && !TARG_DUAL_CORE_ENABLED()
    { .fn = Task_UpdateBaro, .pArg = NULL, .pName = "Baro Update" },
#endif

#if TARG_GPS_ENABLED() && !TARG_DUAL_CORE_ENABLED()
    { .fn = Task_UpdateGps, .pArg = NULL, .pName = "GPS Update" },
#endif

    { .fn = Task_UpdateAttitude, .pArg = NULL, .pName = "Attitude Update" },

#if TARG_BARO_ENABLED() && !TARG_DUAL_CORE_ENABLED()
    { .fn = Task_UpdateAltitude, .pArg = NULL, .pName = "Altitude Update" },
#endif

#if (TARG_MAG_ENABLED() || TARG_GPS_ENABLED()) && !TARG_DUAL_CORE_ENABLED()
    { .fn = Task_UpdateHeading, .pArg = NULL, .pName = "Heading Update" },
#endif

    { .fn = Task_UpdatePid, .pArg = NULL, .pName = "PID Update" },
    { .fn = Task_UpdateMixer, .pArg = NULL, .pName = "Mixer Update" },
};
FJ_DEFINE_SHARED (uint8_t, e_nPrimaryTasks) = sizeof (e_PrimaryTasks) / sizeof (e_PrimaryTasks[0]);

#if !TARG_DUAL_CORE_ENABLED()
FJ_DEFINE_SHARED (Task_t, e_SecondaryTasks[1U]);
FJ_DEFINE_SHARED (uint8_t, e_nSecondaryTasks) = 0U;
#else
FJ_DEFINE_SHARED (Task_t, e_SecondaryTasks[]) = {
    { .fn = Task_UpdateSecondary, .pArg = NULL, .pName = "Secondary Update" },

#if TARG_MAG_ENABLED()
    { .fn = Task_UpdateMag, .pArg = NULL, .pName = "Mag Update" },
#endif

#if TARG_BARO_ENABLED()
    { .fn = Task_UpdateBaro, .pArg = NULL, .pName = "Baro Update" },
#endif

#if TARG_GPS_ENABLED()
    { .fn = Task_UpdateGps, .pArg = NULL, .pName = "GPS Update" },
#endif
#if TARG_BARO_ENABLED()
    { .fn = Task_UpdateAltitude, .pArg = NULL, .pName = "Altitude Update" },
#endif

#if TARG_MAG_ENABLED() || TARG_GPS_ENABLED()
    { .fn = Task_UpdateHeading, .pArg = NULL, .pName = "Heading Update" },
#endif
};
FJ_DEFINE_SHARED (uint8_t, e_nSecondaryTasks) = sizeof (e_SecondaryTasks) / sizeof (e_SecondaryTasks[0]);
#endif

eSTATUS_t Task_UpdateMain (Task_t* pSelf, uint32_t currentTimeUs) {

    FJ_UNUSED (pSelf);
    // TODO
    // update data from rx controller
    // update flight data (target throttle & attitude....) using rx controller input
    // update flight data (attitude, altitude) using sensor data
    // update pid
    // return Pid_Update (currentTimeUs);

    return eSTATUS_FAIL;
}

eSTATUS_t Task_UpdateSecondary (Task_t* pSelf, uint32_t currentTimeUs) {
    // TODO
    return eSTATUS_FAIL;
}

eSTATUS_t Task_UpdateAcc (Task_t* pSelf, uint32_t currentTimeUs) {

    FJ_UNUSED (pSelf);
    return Acc_Update (currentTimeUs, false);
}

eSTATUS_t Task_UpdateGyro (Task_t* pSelf, uint32_t currentTimeUs) {

    FJ_UNUSED (pSelf);
    return Gyro_Update (currentTimeUs, false);
}

eSTATUS_t Task_UpdateMag (Task_t* pSelf, uint32_t currentTimeUs) {

    FJ_UNUSED (pSelf);
    return Mag_Update (currentTimeUs, false);
}

eSTATUS_t Task_UpdateBaro (Task_t* pSelf, uint32_t currentTimeUs) {

    // TODO
    return eSTATUS_FAIL;
}

eSTATUS_t Task_UpdateGps (Task_t* pSelf, uint32_t currentTimeUs) {

    // TODO
    return eSTATUS_FAIL;
}

eSTATUS_t Task_UpdateAttitude (Task_t* pSelf, uint32_t currentTimeUs) {

    FJ_UNUSED (pSelf);
    return Attitude_Update (currentTimeUs);
}

eSTATUS_t Task_UpdateAltitude (Task_t* pSelf, uint32_t currentTimeUs) {

    FJ_UNUSED (pSelf);
    return Altitude_Update (currentTimeUs);
}

eSTATUS_t Task_UpdateHeading (Task_t* pSelf, uint32_t currentTimeUs) {
    // TODO
    return eSTATUS_FAIL;
}

eSTATUS_t Task_UpdatePid (Task_t* pSelf, uint32_t currentTimeUs) {

    FJ_UNUSED (pSelf);
    return Pid_Update (currentTimeUs);
}

eSTATUS_t Task_UpdateMixer (Task_t* pSelf, uint32_t currentTimeUs) {

    FJ_UNUSED (pSelf);
    return Mixer_Mix (currentTimeUs) | Mixer_Update (currentTimeUs);
}