#ifndef DRIVERS_TIM_H
#define DRIVERS_TIM_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "drivers/io/gpio_defs.h"

#include "drivers/tim_defs.h"

#include "targets/target.h"

FJ_DECLARE_SHARED (TimDevice_t, e_TimDevices[]);
FJ_DECLARE_SHARED (TimHwCfg_t, e_TimHwCfgs[]);
FJ_DECLARE_SHARED (uint8_t, e_nTimDevices);

TimDevice_t* Tim_GetById (eTIM_DEVICE_ID_t devId);
TimHwCfg_t* Tim_GetHwCfgById (eTIM_DEVICE_ID_t devId);

/*
 *
 * Platform Tim Device
 *
 */
void Plat_Tim_SetPrescaler (TimDevice_t* pTimDev, uint32_t prescaler);
void Plat_Tim_SetPeriod (TimDevice_t* pTimDev, uint32_t period);
void Plat_Tim_SetCNT (TimDevice_t* pTimDev, uint32_t count);

uint32_t Plat_Tim_GetPrescaler (TimDevice_t* pTimDev);
uint32_t Plat_Tim_GetPeriod (TimDevice_t* pTimDev);
uint32_t Plat_TimDev_GetCNT (TimDevice_t* pTimDev);

uint32_t Plat_Tim_GetClkFreqHz (TimDevice_t* pTimDev);
void Plat_Tim_SetInterruptEnabled (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag, bool enabled);
void Plat_TimDev_ClearInterruptFlag (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag);
uint32_t Plat_TimDev_GetInterruptFlag (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag);
eSTATUS_t Plat_Tim_Init (TimDevCfg_t* const pCfg, TimDevice_t* pOutTimDev);

/*
 *
 * Platform Tim Channel
 *
 */

uint32_t Plat_TimChan_GetCC (TimChannel_t* pTimChan);
void Plat_TimChan_SetCC (TimChannel_t* pTimChan, uint32_t pulseWidth);
eSTATUS_t Plat_TimChan_Init (TimChanCfg_t const* pChanCfg, TimChannel_t* pOutChannel);
eSTATUS_t Plat_TimChan_Start (TimChannel_t* pChannel, uint8_t const* pData, uint32_t size);
eSTATUS_t Plat_TimChan_Stop (TimChannel_t* pChannel);


/*
 *
 * Tim Device Functions
 *
 */
// TimHwCfg_t* TimDev_Get_HwCfg (eTIM_DEVICE_ID_t devId);
TimDmaReqMap_t* TimDev_Get_DmaReqMap (eTIM_DEVICE_ID_t devId);
static inline void TimDev_SetPrescaler (TimDevice_t* pTimDev, uint32_t prescaler) {
    Plat_Tim_SetPrescaler (pTimDev, prescaler);
}

static inline void TimDev_SetPeriod (TimDevice_t* pTimDev, uint32_t period) {
    Plat_Tim_SetPeriod (pTimDev, period);
}

static inline void TimDev_SetCNT (TimDevice_t* pTimDev, uint32_t count) {
    Plat_Tim_SetCNT (pTimDev, count);
}

static inline uint32_t TimDev_GetPrescaler (TimDevice_t* pTimDev) {
    return Plat_Tim_GetPrescaler (pTimDev);
}

static inline uint32_t TimDev_GetPeriod (TimDevice_t* pTimDev) {
    return Plat_Tim_GetPeriod (pTimDev);
}

static inline uint32_t TimDev_GetCNT (TimDevice_t* pTimDev) {
    return Plat_TimDev_GetCNT (pTimDev);
}

static inline uint32_t TimDev_GetClkFreqHz (TimDevice_t* pTimDev) {
    return Plat_Tim_GetClkFreqHz (pTimDev);
}

static inline void TimDev_SetInterruptEnabled (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag, bool enabled) {
    Plat_Tim_SetInterruptEnabled (pTimDev, flag, enabled);
}

static inline void TimDev_ClearInterruptFlag (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag) {
    Plat_TimDev_ClearInterruptFlag (pTimDev, flag);
}

static inline uint32_t TimDev_GetInterruptFlag (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag) {
    return Plat_TimDev_GetInterruptFlag (pTimDev, flag);
}

static inline TimDmaReqMap_t* TimDev_Get_DmaReqMap (eTIM_DEVICE_ID_t devId) {
    return Plat_TimDev_Get_DmaReqMap (devId);
}

static inline eTIM_DEVICE_ID_t TimDev_GetId (TimDevice_t* pTimDev) {

    if (!pTimDev) {
        return eTIM_NULL_DEVICE_ID;
    }
    return pTimDev->id;
}

bool TimDev_HasDmaSupport (eTIM_DEVICE_ID_t timId);
TimDevice_t* TimDev_GetByCaps (bool supportsDma);
TimDevice_t* TimDev_GetById (eTIM_DEVICE_ID_t devId);
bool TimDev_IsInterruptFlagSet (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag);
void TimDev_SetPWMPeriod (TimDevice_t* pTimDev, uint32_t targetClkHz, uint32_t targetHz);
void TimDev_GetBestClkAndPeriod (TimDevice_t* pTimDev, uint32_t targetHz, uint32_t* pOutPrescaler, uint32_t* pOutPeriod);
void TimDev_SetBestClkAndPeriod (TimDevice_t* pTimDev, uint32_t targetHz);

static inline eSTATUS_t TimDev_InitBase (TimDevice_t* pOutTimDev) {

    if (!pOutTimDev) {
        return eSTATUS_NULL_ARG;
    }

    TimDevCfg_t cfg = {
        .id          = pOutTimDev->id,
        .modeType    = eTIM_MODE_BASE,
        .irqPriority = 8U,
    };
    return Plat_Tim_Init (&cfg, pOutTimDev);
}

/*
 *
 * Tim Channel Functions
 *
 */
TimId_u TimChan_GetByGpioId (eGPIO_ID_t gpioId);
eSTATUS_t TimChan_InitCC (eGPIO_ID_t gpioId, TimChannel_t* pOutTimChan);

static inline uint32_t TimChan_GetCC (TimChannel_t* pTimChan) {
    return Plat_TimChan_GetCC (pTimChan);
}

static inline void TimChan_SetCC (TimChannel_t* pTimChan, uint32_t pulseWidth) {
    Plat_TimChan_SetCC (pTimChan, pulseWidth);
}

static inline eSTATUS_t TimChan_Start (TimChannel_t* pTimChan, uint8_t const* pData, uint32_t size) {
    return Plat_TimChan_Start (pTimChan, pData, size);
}

static inline eSTATUS_t TimChan_Stop (TimChannel_t* pTimChan) {
    return Plat_TimChan_Stop (pTimChan);
}

#endif // DRIVERS_TIM_H