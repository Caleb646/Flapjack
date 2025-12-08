#ifndef DRIVERS_TIM_H
#define DRIVERS_TIM_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "drivers/driver.h"

#include "drivers/io/gpio_defs.h"

#include "drivers/tim_defs.h"

#include "targets/target.h"

DRIVER_DECLARE_ARRAY (TimBaseDevice_t*, TimBaseDevices, TARG_MAX_TIMS);

TimHwCfg_t Plat_Tim_Get_DeviceHwCfg (eTIM_ID_t timId);
TimDmaReqMap_t Plat_Tim_Get_DmaReqMap (eTIM_ID_t timId);
void Plat_Tim_SetPrescaler (TimDevice_t* pTimDev, uint32_t prescaler);
void Plat_Tim_SetPeriod (TimDevice_t* pTimDev, uint32_t period);
void Plat_Tim_SetCC (TimDevice_t* pTimDev, uint32_t pulseWidth);
void Plat_Tim_SetCNT (TimDevice_t* pTimDev, uint32_t count);
uint32_t Plat_Tim_GetPrescaler (TimDevice_t* pTimDev);
uint32_t Plat_Tim_GetPeriod (TimDevice_t* pTimDev);
uint32_t Plat_Tim_GetCC (TimDevice_t* pTimDev);
uint32_t Plat_Tim_GetCNT (TimDevice_t* pTimDev);
uint32_t Plat_Tim_GetClkFreqHz (TimDevice_t* pTimDev);
void Plat_Tim_SetInterruptEnabled (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag, bool enabled);
void Plat_Tim_ClearInterruptFlag (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag);
uint32_t Plat_Tim_GetInterruptFlag (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag);
TimBaseDevice_t* Plat_Tim_InitBaseDevice (eTIM_ID_t timId, TimCfg_t* pTimCfg);
eSTATUS_t Plat_Tim_InitCC (TimDevice_t* pTimDev);
eSTATUS_t Plat_Tim_Start (TimDevice_t* pTimDev, uint8_t const* pData, uint32_t size);
eSTATUS_t Plat_Tim_Stop (TimDevice_t* pTimDev);


static inline void Tim_SetPrescaler (TimDevice_t* pTimDev, uint32_t prescaler) {
    Plat_Tim_SetPrescaler (pTimDev, prescaler);
}

static inline void Tim_SetPeriod (TimDevice_t* pTimDev, uint32_t period) {
    Plat_Tim_SetPeriod (pTimDev, period);
}

static inline void Tim_SetCC (TimDevice_t* pTimDev, uint32_t pulseWidth) {
    Plat_Tim_SetCC (pTimDev, pulseWidth);
}

static inline void Tim_SetCNT (TimDevice_t* pTimDev, uint32_t count) {
    Plat_Tim_SetCNT (pTimDev, count);
}

static inline uint32_t Tim_GetPrescaler (TimDevice_t* pTimDev) {
    return Plat_Tim_GetPrescaler (pTimDev);
}

static inline uint32_t Tim_GetPeriod (TimDevice_t* pTimDev) {
    return Plat_Tim_GetPeriod (pTimDev);
}

static inline uint32_t Tim_GetCC (TimDevice_t* pTimDev) {
    return Plat_Tim_GetCC (pTimDev);
}

static inline uint32_t Tim_GetCNT (TimDevice_t* pTimDev) {
    return Plat_Tim_GetCNT (pTimDev);
}

static inline uint32_t Tim_GetClkFreqHz (TimDevice_t* pTimDev) {
    return Plat_Tim_GetClkFreqHz (pTimDev);
}

static inline void Tim_SetInterruptEnabled (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag, bool enabled) {
    Plat_Tim_SetInterruptEnabled (pTimDev, flag, enabled);
}

static inline void Tim_ClearInterruptFlag (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag) {
    Plat_Tim_ClearInterruptFlag (pTimDev, flag);
}

static inline uint32_t Tim_GetInterruptFlag (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag) {
    return Plat_Tim_GetInterruptFlag (pTimDev, flag);
}

static inline TimDmaReqMap_t Tim_Get_DmaReqMap (eTIM_ID_t timId) {
    return Plat_Tim_Get_DmaReqMap (timId);
}

bool Tim_HasDmaSupport (eTIM_ID_t timId);
eTIM_DEVICE_ID_t Tim_Alloc (bool supportsDma);
bool Tim_IsInterruptFlagSet (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag);
void Tim_SetupPWMPeriod (TimDevice_t* pTimDev, uint32_t targetClkHz, uint32_t targetHz);
eSTATUS_t Tim_Init (TimCfg_t* pTimCfg, TimDevice_t* pOutTimDev);

static inline eSTATUS_t Tim_Start (TimDevice_t* pTimDev, uint8_t const* pData, uint32_t size) {
    return Plat_Tim_Start (pTimDev, pData, size);
}

static inline eSTATUS_t Tim_Stop (TimDevice_t* pTimDev) {
    return Plat_Tim_Stop (pTimDev);
}

#endif // DRIVERS_TIM_H