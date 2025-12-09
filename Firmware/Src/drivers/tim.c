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

DRIVER_DEFINE_ARRAY (TimDevice_t*, TimDevices, TARG_MAX_TIMS);

bool TimDev_HasDmaSupport (eTIM_DEVICE_ID_t timId) {
    TimDmaReqMap_t dmaReqMap = TimDev_Get_DmaReqMap (timId);
    return (!dmaReqMap.cc1 && !dmaReqMap.cc2 && !dmaReqMap.cc3 && !dmaReqMap.cc4 && !dmaReqMap.update);
}

eTIM_DEVICE_ID_t TimDev_FindAvailable (bool supportsDma) {

    for (eTIM_DEVICE_ID_t devId = eTIM_1_DEVICE_ID; devId <= TARG_MAX_TIMS; ++devId) {

        TimDevice_t** ppTimBaseDev = TimDevices_GetMutable (TIM_DEV_ID_TO_INDEX (devId));
        if (!ppTimBaseDev || *ppTimBaseDev) {
            continue;
        }

        if (supportsDma && !TimDev_HasDmaSupport (devId)) {
            continue;
        }

        return devId;
    }
    return eTIM_NULL_DEVICE_ID;
}

bool TimDev_IsInterruptFlagSet (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag) {
    return (TimDev_GetInterruptFlag (pTimDev, flag) != 0);
}

void TimDev_SetupPWMPeriod (TimDevice_t* pTimDev, uint32_t targetClkHz, uint32_t targetHz) {

    if (!pTimDev || !targetClkHz || !targetHz) {
        return;
    }

    uint32_t prescaler = (TimDev_GetClkFreqHz (pTimDev) / targetClkHz) - 1U;
    uint32_t period    = (targetClkHz / targetHz) - 1U;

    TimDev_SetPrescaler (pTimDev, prescaler);
    TimDev_SetPeriod (pTimDev, period);
}

eSTATUS_t TimChan_Init (TimCfg_t const* pTimCfg, TimChannel_t* pOutTimChan) {

    if (!pTimCfg || !pOutTimChan) {
        return eSTATUS_NULL_ARG;
    }

    TimDevice_t* pTimBaseDev = Plat_TimDev_Init (pTimCfg->devId, pTimCfg);
    if (!pTimBaseDev) {
        LOG_ERROR ("Failed to initialize platform timer device handle");
        return eSTATUS_FAILURE;
    }

    TimHwCfg_t hwCfg = Plat_TimDev_Get_HwCfg (pTimCfg->devId);
    if (!GPIO_Init (pTimCfg->gpioId, pTimCfg->chanId, PLAT_GPIO_CFG_TIM_PWM, hwCfg.gpioAf)) {
        return eSTATUS_FAILURE;
    }

    pOutTimChan->devId       = pTimCfg->devId;
    pOutTimChan->chanId      = pTimCfg->chanId;
    pOutTimChan->modeType    = pTimCfg->modeType;
    pOutTimChan->irqNum      = hwCfg.irqNum;
    pOutTimChan->irqPriority = pTimCfg->irqPriority;
    pOutTimChan->pTimBaseDev = pTimBaseDev;

    eSTATUS_t status = Plat_TimChan_InitCC (pOutTimChan);
    RETURN_IF (FJ_FAIL (status), status, "Failed to initialize platform timer CC");

    return eSTATUS_SUCCESS;
}