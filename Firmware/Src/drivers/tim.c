#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "core/core.h"

#include "drivers/tim.h"
#include "drivers/tim_defs.h"

#include "drivers/io/gpio.h"
#include "drivers/io/gpio_defs.h"

#include "platform/platform.h"

#include "targets/target.h"

DRIVER_DEFINE_ARRAY (TimBaseDevice_t*, TimBaseDevices, TARG_MAX_TIMS);

bool Tim_IsInterruptFlagSet (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag) {
    return (Tim_GetInterruptFlag (pTimDev, flag) != 0);
}

void Tim_SetupPWMPeriod (TimDevice_t* pTimDev, uint32_t targetClkHz, uint32_t targetHz) {

    if (!pTimDev || !targetClkHz || !targetHz) {
        return;
    }

    uint32_t prescaler = (Tim_GetClkFreqHz (pTimDev) / targetClkHz) - 1U;
    uint32_t period    = (targetClkHz / targetHz) - 1U;

    Tim_SetPrescaler (pTimDev, prescaler);
    Tim_SetPeriod (pTimDev, period);
}

eSTATUS_t Tim_Init (TimCfg_t* pTimCfg, TimDevice_t* pOutTimDev) {

    if (!pTimCfg || !pOutTimDev) {
        return eSTATUS_NULL_ARG;
    }

    TimBaseDevice_t* pTimBaseDev = Plat_Tim_InitBaseDevice (pTimCfg->id, pTimCfg);
    if (!pTimBaseDev) {
        LOG_ERROR ("Failed to initialize platform timer device handle");
        return eSTATUS_FAILURE;
    }

    TimHwCfg_t hwCfg = Plat_Tim_Get_DeviceHwCfg (pTimCfg->id);
    if (!GPIO_Init (pTimCfg->gpioId, pTimCfg->id, PLAT_GPIO_CFG_TIM_PWM, hwCfg.gpioAf)) {
        return eSTATUS_FAILURE;
    }

    pOutTimDev->id          = pTimCfg->id;
    pOutTimDev->modeType    = pTimCfg->modeType;
    pOutTimDev->irqNum      = hwCfg.irqNum;
    pOutTimDev->irqPriority = pTimCfg->irqPriority;
    pOutTimDev->pTimBaseDev = pTimBaseDev;

    eSTATUS_t status = Plat_Tim_InitCC (pOutTimDev);
    RETURN_IF (FJ_FAIL (status), status, "Failed to initialize platform timer CC");

    return eSTATUS_SUCCESS;
}