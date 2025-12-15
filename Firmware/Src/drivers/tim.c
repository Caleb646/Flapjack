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

TimHwCfg_t* TimDev_Get_HwCfg (eTIM_DEVICE_ID_t devId) {

    for (uint32_t i = 0; i < e_nTimHwCfgs; ++i) {
        if (e_TimHwCfgs[i].id == devId) {
            return &e_TimHwCfgs[i];
        }
    }
    return NULL;
}

TimDmaReqMap_t* TimDev_Get_DmaReqMap (eTIM_DEVICE_ID_t devId) {

    for (uint32_t i = 0; i < e_nTimDmaReqMaps; ++i) {
        if (e_TimDmaReqMaps[i].id == devId) {
            return &e_TimDmaReqMaps[i];
        }
    }
    return NULL;
}

bool TimDev_HasDmaSupport (eTIM_DEVICE_ID_t timId) {
    return TimDev_Get_DmaReqMap (timId) != NULL;
}

TimDevice_t* TimDev_GetByCaps (bool supportsDma) {

    for (eTIM_DEVICE_ID_t devId = eTIM_1_DEVICE_ID; devId <= TARG_MAX_TIMS; ++devId) {

        if (supportsDma && !TimDev_HasDmaSupport (devId)) {
            continue;
        }

        return TimDev_GetById (devId);
    }
    return NULL;
}

TimDevice_t* TimDev_GetById (eTIM_DEVICE_ID_t devId) {

    TimDevice_t** ppTimDev = TimDevices_GetMutable (devId);
    if (!ppTimDev) {
        return NULL;
    }

    // already allocated
    if (*ppTimDev) {
        return *ppTimDev;
    }

    *ppTimDev = Alloc_SharedMem (sizeof (TimDevice_t));
    // failed to allocate
    if (!*ppTimDev) {
        return NULL;
    }

    (*ppTimDev)->id = devId;
    return *ppTimDev;
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

void TimDev_GetBestClkAndPeriod (TimDevice_t* pTimDev, uint32_t targetHz, uint32_t* pOutPrescaler, uint32_t* pOutPeriod) {

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
    if (pOutPrescaler) {
        *pOutPrescaler = bestPrescaler;
    }
    if (pOutPeriod) {
        *pOutPeriod = bestPeriod;
    }
}

void TimDev_SetBestClkAndPeriod (TimDevice_t* pTimDev, uint32_t targetHz) {

    uint32_t bestPrescaler = 0U;
    uint32_t bestPeriod    = 0U;
    TimDev_GetBestClkAndPeriod (pTimDev, targetHz, &bestPrescaler, &bestPeriod);
    TimDev_SetPrescaler (pTimDev, bestPrescaler);
    TimDev_SetPeriod (pTimDev, bestPeriod);
}

TimId_u TimChan_GetByGpioId (eGPIO_ID_t gpioId) {

    for (uint32_t i = 0; i < e_nTimHwCfgs; ++i) {
        for (uint32_t chanIdx = 0; chanIdx < 4U; ++chanIdx) {
            if (e_TimHwCfgs[i].channelGpioIds[chanIdx] == gpioId) {
                return (TimId_u){ .chanId = TIM_CHAN_INDEX_TO_ID (chanIdx), .devId = e_TimHwCfgs[i].id };
            }
        }
    }
    return (TimId_u){ .chanId = eTIM_CHANNEL_NULL_ID, .devId = eTIM_NULL_DEVICE_ID };
}

eSTATUS_t TimChan_InitCC (eGPIO_ID_t gpioId, TimChannel_t* pOutTimChan) {

    if (!gpioId || !pOutTimChan) {
        return eSTATUS_NULL_ARG;
    }

    eSTATUS_t status = eSTATUS_SUCCESS;
    TimId_u timId    = TimChan_GetByGpioId (gpioId);
    if (timId.devId == eTIM_NULL_DEVICE_ID || timId.chanId == eTIM_CHANNEL_NULL_ID) {
        return eSTATUS_FAILURE;
    }

    TimChanCfg_t chanCfg = {
        .id     = timId.chanId,
        .gpioId = gpioId,
        .devCfg = {
            .id          = timId.devId,
            .modeType    = eTIM_MODE_PWM,
            .irqPriority = 7U,
        },
    };

    status = Plat_TimChan_Init (&chanCfg, pOutTimChan);
    RETURN_IF (FJ_FAIL (status), status, "Failed to initialize platform timer CC");

    return eSTATUS_SUCCESS;
}