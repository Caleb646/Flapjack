#include "device/mag.h"
#include "drivers/sensors/mag/mag.h"
#include "umsg_sensors.h"

eSTATUS_t SensorMag_Init(void) {
    return Mag_Init ();
}

eSTATUS_t SensorMag_Update(void) {
    Mag_t* pMag = Mag_Get();
    if (!pMag) {
        return eSTATUS_FAILURE;
    }

    eSTATUS_t status = Mag_Update_(pMag, false, &pMag->normedData);
    if (STATUS_FAIL(status)) {
        return status;
    }

    umsg_sensors_mag_t msg = {
        .field = { pMag->normedData.x, pMag->normedData.y, pMag->normedData.z },
    };
    umsg_sensors_mag_publish(&msg);
    return eSTATUS_SUCCESS;
}
