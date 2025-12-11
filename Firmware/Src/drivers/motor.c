#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "core/core.h"

#include "drivers/dma.h"
#include "drivers/dma_defs.h"
#include "drivers/motor.h"
#include "drivers/tim.h"
#include "drivers/tim_defs.h"

#include "drivers/io/gpio.h"
#include "drivers/io/gpio_defs.h"

#include "cfg/motor.h"

#include "platform/platform.h"

#include "targets/target.h"

typedef struct Motors_s {
    MotorProtVtbl_t vtbl;
    float outputs[TARG_MAX_MOTORS];
    uint8_t nMotors;
} Motors_t;

typedef struct Servos_s {
    MotorProtVtbl_t vtbl;
    float outputs[TARG_MAX_SERVOS];
    uint8_t nServos;
} Servos_t;

static TARG_SHARED_MEM_SECTION Motors_t g_Motors = { 0 };
static TARG_SHARED_MEM_SECTION Servos_t g_Servos = { 0 };

extern eSTATUS_t Plat_Dshot_Init (MotorsCfg_t const* pCfg, MotorProtVtbl_t* pOutVtbl);

static eSTATUS_t Dshot_Init (MotorsCfg_t const* pCfg, MotorProtVtbl_t* pOutVtbl) {
    return Plat_Dshot_Init (pCfg, pOutVtbl);
}

eSTATUS_t Motors_Init (void) {

    eSTATUS_t status        = eSTATUS_FAILURE;
    Motors_t* pMotors       = &g_Motors;
    MotorsCfg_t const* pCfg = MotorsCfg_Get ();
    if (!pCfg || !pMotors) {
        return eSTATUS_FAILURE;
    }

    if (MOTOR_PROT_IS_DSHOT (pCfg->protType)) {
        status = Dshot_Init (pCfg, &pMotors->vtbl);
    }

    LOG_ERROR_IF (FJ_FAIL (status), "Failed to init motors");
    return status;
}

eSTATUS_t Motors_Arm (void) {
    // Implementation of Motors_Arm function
    // ...
    return eSTATUS_SUCCESS;
}

eSTATUS_t Motors_Mix (void) {
    // Implementation of Motors_Mix function
    // ...
    return eSTATUS_SUCCESS;
}

eSTATUS_t Motors_Update (void) {
    // Implementation of Motors_Update function
    // ...
    return eSTATUS_SUCCESS;
}