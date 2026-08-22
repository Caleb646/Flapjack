#include "devices/motors.h"

#include "drivers/motor/motordrv.h"

#include "core/core.h"

#include <stdbool.h>
#include <string.h>

eSTATUS_t Motors_Init (Motors_t* pOutMotors) {

    memset (pOutMotors, 0, sizeof (Motors_t));
    return MotorDrv_Init (&pOutMotors->drv);
}

eSTATUS_t Motors_Write (Motors_t* pMotors, float throttles[BRD_MOTOR_COUNT]) {

    if (!pMotors->armed) {
        LOG_ERROR ("Cannot write to motors because they are not armed");
        return eSTATUS_FAILURE;
    }
    return pMotors->drv.Write (pMotors->drv.ctx, throttles);
}

eSTATUS_t Motors_Arm (Motors_t* pMotors) {

    /* BUSY means the arm handshake is under way and wants calling again, not
     * that anything went wrong - so it is checked before STATUS_FAIL, which
     * treats every non-success alike. */
    eSTATUS_t status = pMotors->drv.Arm (pMotors->drv.ctx);
    if (status == eSTATUS_BUSY) {
        return status;
    }
    if (STATUS_FAIL (status)) {
        LOG_ERROR ("Failed to arm motor");
        return eSTATUS_FAILURE;
    }
    pMotors->armed = true;
    LOG_INFO ("Motors armed");
    return eSTATUS_SUCCESS;
}

eSTATUS_t Motors_Disarm (Motors_t* pMotors) {

    pMotors->armed = false;
    LOG_INFO ("Motors disarmed");
    return pMotors->drv.Disarm (pMotors->drv.ctx);
}
