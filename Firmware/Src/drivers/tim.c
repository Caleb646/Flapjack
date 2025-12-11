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

TimDevice_t* TimDev_AllocByCaps (bool supportsDma) {

    for (eTIM_DEVICE_ID_t devId = eTIM_1_DEVICE_ID; devId <= TARG_MAX_TIMS; ++devId) {

        TimDevice_t** ppTimBaseDev = TimDevices_GetMutable (TIM_DEV_ID_TO_INDEX (devId));
        if (!ppTimBaseDev || *ppTimBaseDev) {
            continue;
        }

        if (supportsDma && !TimDev_HasDmaSupport (devId)) {
            continue;
        }

        return TimDev_AllocById (devId);
    }
    return NULL;
}

TimDevice_t* TimDev_AllocById (eTIM_DEVICE_ID_t devId) {

    TimDevice_t** ppTimBaseDev = TimDevices_GetMutable (TIM_DEV_ID_TO_INDEX (devId));
    if (*ppTimBaseDev) {
        return NULL; // Already allocated
    }

    *ppTimBaseDev = Alloc_SharedMem (sizeof (TimDevice_t));
    if (*ppTimBaseDev) {
        (*ppTimBaseDev)->id = devId;
    }

    return *ppTimBaseDev;
}

TimDevice_t* TimDev_GetOrAllocById (eTIM_DEVICE_ID_t devId) {

    TimDevice_t** ppTimBaseDev = TimDevices_GetMutable (TIM_DEV_ID_TO_INDEX (devId));
    if (*ppTimBaseDev) {
        return *ppTimBaseDev;
    }
    return TimDev_AllocById (devId);
}

bool TimDev_IsInterruptFlagSet (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag) {
    return (TimDev_GetInterruptFlag (pTimDev, flag) != 0);
}

void TimDev_SetPWMPeriod (TimDevice_t* pTimDev, uint32_t targetClkHz, uint32_t targetHz) {

    if (!pTimDev || !targetClkHz || !targetHz) {
        return;
    }

    uint32_t prescaler = (TimDev_GetClkFreqHz (pTimDev) / targetClkHz) - 1U;
    uint32_t period    = (targetClkHz / targetHz) - 1U;

    TimDev_SetPrescaler (pTimDev, prescaler);
    TimDev_SetPeriod (pTimDev, period);
}

void TimDev_SetBestClkAndPeriod (TimDevice_t* pTimDev, uint32_t targetHz) {

    if (!pTimDev || !targetHz) {
        return;
    }
    uint32_t sysClkHz      = Plat_GetSysClkFreqHz ();
    uint32_t bestPrescaler = 0U;
    uint32_t bestPeriod    = 0U;
    uint32_t bestDiff      = UINT32_MAX;
    for (uint32_t prescaler = 0U; prescaler <= 0xFFFFU; ++prescaler) {
        uint32_t clkHz  = sysClkHz / (prescaler + 1U);
        uint32_t period = clkHz / targetHz;
        if (period == 0U || period > 0xFFFFU) {
            continue;
        }
        uint32_t actualHz = clkHz / period;
        uint32_t diff     = (actualHz > targetHz) ? (actualHz - targetHz) : (targetHz - actualHz);
        if (diff < bestDiff) {
            bestDiff      = diff;
            bestPrescaler = prescaler;
            bestPeriod    = period;
            if (bestDiff == 0U) {
                break;
            }
        }
    }
    TimDev_SetPrescaler (pTimDev, bestPrescaler);
    TimDev_SetPeriod (pTimDev, bestPeriod);
}

eSTATUS_t TimChan_Init (TimCfg_t const* pTimCfg, TimChannel_t* pOutTimChan) {

    if (!pTimCfg || !pOutTimChan) {
        return eSTATUS_NULL_ARG;
    }

    TimDevice_t* pTimBaseDev = TimDev_GetOrAllocById (pTimCfg->devId);
    eSTATUS_t status         = Plat_TimDev_Init (pTimBaseDev);
    RETURN_IF (FJ_FAIL (status), status, "Failed to initialize platform timer device");

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