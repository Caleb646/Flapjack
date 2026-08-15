/*
 * Simulation servo backend: instead of driving PWM hardware, convert each
 * commanded pulse width to a tilt angle and stream it to the JSBSim bridge
 * over the sim link.
 */

#include "drivers/servo/servodrv.h"

#include "core/core.h"

#include "devices/servos.h"   // SERVO_*_US_DC calibration
#include "drivers/sim_link/sim_link.h"

#include <string.h>

/* Tilt servo mechanical travel: full deflection (SERVO_LEFT/RIGHT_US_DC) maps to
 * +/- 90 deg about the centre pulse width. */
#define SERVO_TILT_RANGE_RAD 1.5707963F

static float UsToAngleRad (uint16_t us) {
    float norm = ((float)us - (float)SERVO_CENTER_US_DC) /
                 ((float)SERVO_RIGHT_US_DC - (float)SERVO_CENTER_US_DC);
    return norm * SERVO_TILT_RANGE_RAD;
}

STATIC eSTATUS_t Sim_Write (void* ctx, uint16_t const us[BRD_SERVO_COUNT]) {

    FJ_UNUSED (ctx);
    float angles[BRD_SERVO_COUNT];
    for (uint32_t i = 0; i < BRD_SERVO_COUNT; ++i) {
        angles[i] = UsToAngleRad (us[i]);
    }
    return SimLink_SendServos (angles, BRD_SERVO_COUNT);
}

eSTATUS_t ServoDrv_Init (ServoDriver_t* pOutDriver) {

    if (!pOutDriver) {
        return eSTATUS_NULL_ARG;
    }
    memset (pOutDriver, 0, sizeof (ServoDriver_t));
    pOutDriver->ctx   = NULL;
    pOutDriver->Write = Sim_Write;
    return eSTATUS_SUCCESS;
}
