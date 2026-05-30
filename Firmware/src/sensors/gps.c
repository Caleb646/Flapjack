#include "sensors/gps.h"
#include "drivers/sensors/gps/gps.h"
#include "umsg_sensors.h"

eSTATUS_t SensorGps_Update(void) {
    eSTATUS_t status = Gps_Update();
    if (STATUS_FAIL(status)) {
        return status;
    }

    umsg_sensors_gps_t msg = {
        .lat      = (double)g_Gps.data.latitude,
        .lon      = (double)g_Gps.data.longitude,
        .alt      = g_Gps.data.altitude,
        .speed    = g_Gps.data.speed,
        .course   = g_Gps.data.course,
        .fix_type = g_Gps.data.fixType,
        .sats     = g_Gps.data.satellitesInUse,
    };
    umsg_sensors_gps_publish(&msg);
    return eSTATUS_SUCCESS;
}
