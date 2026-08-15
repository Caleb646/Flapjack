#include "devices/servos.h"

#include "core/core.h"

#include "drivers/servo/servodrv.h"

FJ_DEFINE_SHARED (Servos_t, g_Servos) = { 0 };

eSTATUS_t Servos_Init (void) {
    return ServoDrv_Init (&g_Servos.drv);
}

eSTATUS_t Servos_Write (uint16_t const servoVals[BRD_SERVO_COUNT]) {

    if (!g_Servos.drv.Write) {
        return eSTATUS_FAILURE;
    }
    return g_Servos.drv.Write (g_Servos.drv.ctx, servoVals);
}
