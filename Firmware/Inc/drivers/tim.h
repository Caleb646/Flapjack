#ifndef DRIVERS_TIM_H
#define DRIVERS_TIM_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "drivers/driver.h"

#include "drivers/io/gpio_defs.h"

#include "drivers/tim_defs.h"

#include "targets/target.h"

DRIVER_DECLARE_ARRAY (TimDevice_t*, TimDevices, TARG_MAX_TIMS);

TimHwCfg_t Plat_TimDev_Get_HwCfg (eTIM_DEVICE_ID_t timId);
TimDmaReqMap_t Plat_TimDev_Get_DmaReqMap (eTIM_DEVICE_ID_t timId);
void Plat_TimDev_SetPrescaler (TimDevice_t* pTimDev, uint32_t prescaler);
void Plat_TimDev_SetPeriod (TimDevice_t* pTimDev, uint32_t period);
void Plat_TimDev_SetCNT (TimDevice_t* pTimDev, uint32_t count);

uint32_t Plat_TimDev_GetPrescaler (TimDevice_t* pTimDev);
uint32_t Plat_TimDev_GetPeriod (TimDevice_t* pTimDev);
uint32_t Plat_TimDev_GetCNT (TimDevice_t* pTimDev);

uint32_t Plat_TimChan_GetCC (TimChannel_t* pTimChan);
void Plat_TimChan_SetCC (TimChannel_t* pTimChan, uint32_t pulseWidth);

uint32_t Plat_TimDev_GetClkFreqHz (TimDevice_t* pTimDev);
void Plat_TimDev_SetInterruptEnabled (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag, bool enabled);
void Plat_TimDev_ClearInterruptFlag (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag);
uint32_t Plat_TimDev_GetInterruptFlag (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag);
eSTATUS_t Plat_TimDev_Init (TimDevice_t* pOutTimDev);

eSTATUS_t Plat_TimChan_InitCC (TimChannel_t* pChannel);
eSTATUS_t Plat_TimChan_Start (TimChannel_t* pChannel, uint8_t const* pData, uint32_t size);
eSTATUS_t Plat_TimChan_Stop (TimChannel_t* pChannel);

static inline void TimDev_SetPrescaler (TimDevice_t* pTimDev, uint32_t prescaler) {
    Plat_TimDev_SetPrescaler (pTimDev, prescaler);
}

static inline void TimDev_SetPeriod (TimDevice_t* pTimDev, uint32_t period) {
    Plat_TimDev_SetPeriod (pTimDev, period);
}

static inline void TimChan_SetCC (TimChannel_t* pTimChan, uint32_t pulseWidth) {
    Plat_TimChan_SetCC (pTimChan, pulseWidth);
}

static inline void TimDev_SetCNT (TimDevice_t* pTimDev, uint32_t count) {
    Plat_TimDev_SetCNT (pTimDev, count);
}

static inline uint32_t TimDev_GetPrescaler (TimDevice_t* pTimDev) {
    return Plat_TimDev_GetPrescaler (pTimDev);
}

static inline uint32_t TimDev_GetPeriod (TimDevice_t* pTimDev) {
    return Plat_TimDev_GetPeriod (pTimDev);
}

static inline uint32_t TimChan_GetCC (TimChannel_t* pTimChan) {
    return Plat_TimChan_GetCC (pTimChan);
}

static inline uint32_t TimDev_GetCNT (TimDevice_t* pTimDev) {
    return Plat_TimDev_GetCNT (pTimDev);
}

static inline uint32_t TimDev_GetClkFreqHz (TimDevice_t* pTimDev) {
    return Plat_TimDev_GetClkFreqHz (pTimDev);
}

static inline void TimDev_SetInterruptEnabled (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag, bool enabled) {
    Plat_TimDev_SetInterruptEnabled (pTimDev, flag, enabled);
}

static inline void TimDev_ClearInterruptFlag (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag) {
    Plat_TimDev_ClearInterruptFlag (pTimDev, flag);
}

static inline uint32_t TimDev_GetInterruptFlag (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag) {
    return Plat_TimDev_GetInterruptFlag (pTimDev, flag);
}

static inline TimDmaReqMap_t TimDev_Get_DmaReqMap (eTIM_DEVICE_ID_t timId) {
    return Plat_TimDev_Get_DmaReqMap (timId);
}

static inline eTIM_DEVICE_ID_t TimDev_GetID (TimDevice_t* pTimDev) {

    if (!pTimDev) {
        return eTIM_NULL_DEVICE_ID;
    }
    return pTimDev->id;
}

bool TimDev_HasDmaSupport (eTIM_DEVICE_ID_t timId);
TimDevice_t* TimDev_AllocByCaps (bool supportsDma);
TimDevice_t* TimDev_AllocById (eTIM_DEVICE_ID_t devId);
bool TimDev_IsInterruptFlagSet (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag);
void TimDev_SetPWMPeriod (TimDevice_t* pTimDev, uint32_t targetClkHz, uint32_t targetHz);
void TimDev_SetBestClkAndPeriod (TimDevice_t* pTimDev, uint32_t targetHz);

static inline eSTATUS_t TimDev_InitDefault (TimDevice_t* pOutTimDev) {
    return Plat_TimDev_Init (pOutTimDev);
}
eSTATUS_t TimChan_Init (TimCfg_t const* pTimCfg, TimChannel_t* pOutTimChan);

static inline eSTATUS_t TimChan_Start (TimChannel_t* pTimChan, uint8_t const* pData, uint32_t size) {
    return Plat_TimChan_Start (pTimChan, pData, size);
}

static inline eSTATUS_t TimChan_Stop (TimChannel_t* pTimChan) {
    return Plat_TimChan_Stop (pTimChan);
}

#endif // DRIVERS_TIM_H