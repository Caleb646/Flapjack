#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "core/core.h"

#include "drivers/tim.h"
#include "drivers/tim_defs.h"

#include "drivers/io/gpio.h"
#include "drivers/io/gpio_defs.h"

#include "platform/stm32/platform.h"

#define TIM_ID_TO_HAL_CHAN(TIM_ID)                                         \
    (                                                                      \
    (TIM_ID_TO_CHANNEL_ID (TIM_ID) == eTIM_CHANNEL_1_ID) ? TIM_CHANNEL_1 : \
    (TIM_ID_TO_CHANNEL_ID (TIM_ID) == eTIM_CHANNEL_2_ID) ? TIM_CHANNEL_2 : \
    (TIM_ID_TO_CHANNEL_ID (TIM_ID) == eTIM_CHANNEL_3_ID) ? TIM_CHANNEL_3 : \
    (TIM_ID_TO_CHANNEL_ID (TIM_ID) == eTIM_CHANNEL_4_ID) ? TIM_CHANNEL_4 : \
                                                           0xFFFFU         \
    )

static uint32_t const g_InterruptFlagsMap[] = { [eTIM_INTERRUPT_FLAG_UPDATE]  = TIM_FLAG_UPDATE,
                                                [eTIM_INTERRUPT_FLAG_CC1]     = TIM_FLAG_CC1,
                                                [eTIM_INTERRUPT_FLAG_CC2]     = TIM_FLAG_CC2,
                                                [eTIM_INTERRUPT_FLAG_CC3]     = TIM_FLAG_CC3,
                                                [eTIM_INTERRUPT_FLAG_CC4]     = TIM_FLAG_CC4,
                                                [eTIM_INTERRUPT_FLAG_TRIGGER] = TIM_FLAG_TRIGGER,
                                                [eTIM_INTERRUPT_FLAG_BREAK]   = TIM_FLAG_BREAK,
                                                [eTIM_INTERRUPT_FLAG_COM]     = TIM_FLAG_COM };


void Stm32_Tim_IRQHandler (eTIM_DEVICE_ID_t devId) {

    TimBaseDevice_t** ppTimBaseDev = TimBaseDevices_GetMutable (devId);
    if (!ppTimBaseDev) {
        return;
    }

    TimBaseDevice_t* pTimBaseDev = *ppTimBaseDev;
    if (!pTimBaseDev) {
        return;
    }

    HAL_TIM_IRQHandler (&(pTimBaseDev->handle));
}

#define TIM_DEF_INTERRUPT_HANDLER(TIM_DEV_NUM)                 \
    void TIM##TIM_DEV_NUM##_IRQHandler (void) {                \
        Stm32_Tim_IRQHandler (eTIM_##TIM_DEV_NUM##_DEVICE_ID); \
    }

// TODO: Fix interrupt handler names
TIM_DEF_INTERRUPT_HANDLER (1);
TIM_DEF_INTERRUPT_HANDLER (2);
TIM_DEF_INTERRUPT_HANDLER (5);
// TIM8_CC_IRQHandler
TIM_DEF_INTERRUPT_HANDLER (8);
// TIM8_BRK_TIM12_IRQHandler
TIM_DEF_INTERRUPT_HANDLER (12);
// TIM8_UP_TIM13_IRQHandler
TIM_DEF_INTERRUPT_HANDLER (13);

TimHwCfg_t Plat_Tim_Get_DeviceHwCfg (eTIM_ID_t timId) {

    // __HAL_RCC_TIM2_CLK_ENABLE();
    switch (TIM_ID_TO_DEVICE_ID (timId)) {
    case eTIM_1_DEVICE_ID:
        return (TimHwCfg_t){ .pInstance     = TIM1,
                             .pRccEnableReg = &RCC->APB2ENR,
                             .rccEnableMsk  = RCC_APB2ENR_TIM1EN,
                             .gpioAf        = GPIO_AF1_TIM1,
                             .irqNum        = TIM1_UP_IRQn };
    case eTIM_2_DEVICE_ID:
        return (TimHwCfg_t){ .pInstance     = TIM2,
                             .pRccEnableReg = &RCC->APB1LENR,
                             .rccEnableMsk  = RCC_APB1LENR_TIM2EN,
                             .gpioAf        = GPIO_AF1_TIM2,
                             .irqNum        = TIM2_IRQn };
    case eTIM_5_DEVICE_ID:
        return (TimHwCfg_t){ .pInstance     = TIM5,
                             .pRccEnableReg = &RCC->APB1LENR,
                             .rccEnableMsk  = RCC_APB1LENR_TIM5EN,
                             .gpioAf        = GPIO_AF2_TIM5,
                             .irqNum        = TIM5_IRQn };
    case eTIM_8_DEVICE_ID:
        return (TimHwCfg_t){ .pInstance     = TIM8,
                             .pRccEnableReg = &RCC->APB2ENR,
                             .rccEnableMsk  = RCC_APB2ENR_TIM8EN,
                             .gpioAf        = GPIO_AF3_TIM8,
                             .irqNum        = TIM8_UP_TIM13_IRQn };
    case eTIM_12_DEVICE_ID:
        return (TimHwCfg_t){ .pInstance     = TIM12,
                             .pRccEnableReg = &RCC->APB1LENR,
                             .rccEnableMsk  = RCC_APB1LENR_TIM12EN,
                             .gpioAf        = GPIO_AF2_TIM12,
                             // TIM_8 brk interrupt and TIM_12 global interrupt
                             .irqNum = TIM8_BRK_TIM12_IRQn };

    case eTIM_13_DEVICE_ID:
        return (TimHwCfg_t){ .pInstance     = TIM13,
                             .pRccEnableReg = &RCC->APB1LENR,
                             .rccEnableMsk  = RCC_APB1LENR_TIM13EN,
                             .gpioAf        = GPIO_AF9_TIM13,
                             // TIM_8 update interrupt and TIM_13 global interrupt
                             .irqNum = TIM8_UP_TIM13_IRQn };
    default: return (TimHwCfg_t){ 0 };
    }
}

TimDmaReqMap_t Plat_Tim_Get_DmaReqMap (eTIM_ID_t timId) {

    switch (TIM_ID_TO_DEVICE_ID (timId)) {
    case eTIM_1_DEVICE_ID:
        return (TimDmaReqMap_t){ .cc1    = DMA_REQUEST_TIM1_CH1,
                                 .cc2    = DMA_REQUEST_TIM1_CH2,
                                 .cc3    = DMA_REQUEST_TIM1_CH3,
                                 .cc4    = DMA_REQUEST_TIM1_CH4,
                                 .update = DMA_REQUEST_TIM1_UP };
    case eTIM_2_DEVICE_ID:
        return (TimDmaReqMap_t){ .cc1    = DMA_REQUEST_TIM2_CH1,
                                 .cc2    = DMA_REQUEST_TIM2_CH2,
                                 .cc3    = DMA_REQUEST_TIM2_CH3,
                                 .cc4    = DMA_REQUEST_TIM2_CH4,
                                 .update = DMA_REQUEST_TIM2_UP };
    case eTIM_5_DEVICE_ID:
        return (TimDmaReqMap_t){ .cc1    = DMA_REQUEST_TIM5_CH1,
                                 .cc2    = DMA_REQUEST_TIM5_CH2,
                                 .cc3    = DMA_REQUEST_TIM5_CH3,
                                 .cc4    = DMA_REQUEST_TIM5_CH4,
                                 .update = DMA_REQUEST_TIM5_UP };
    case eTIM_8_DEVICE_ID:
        return (TimDmaReqMap_t){ .cc1    = DMA_REQUEST_TIM8_CH1,
                                 .cc2    = DMA_REQUEST_TIM8_CH2,
                                 .cc3    = DMA_REQUEST_TIM8_CH3,
                                 .cc4    = DMA_REQUEST_TIM8_CH4,
                                 .update = DMA_REQUEST_TIM8_UP };
    default: return (TimDmaReqMap_t){ 0 };
    }
}

void Plat_Tim_SetPrescaler (TimDevice_t* pTimDev, uint32_t prescaler) {

    if (!pTimDev || !pTimDev->pTimBaseDev) {
        return;
    }
    __HAL_TIM_SET_PRESCALER (&(pTimDev->pTimBaseDev->handle), prescaler);
}

void Plat_Tim_SetPeriod (TimDevice_t* pTimDev, uint32_t period) {

    if (!pTimDev || !pTimDev->pTimBaseDev) {
        return;
    }
    __HAL_TIM_SET_AUTORELOAD (&(pTimDev->pTimBaseDev->handle), period);
}

void Plat_Tim_SetCC (TimDevice_t* pTimDev, uint32_t pulseWidth) {

    if (!pTimDev || !pTimDev->pTimBaseDev) {
        return;
    }
    __HAL_TIM_SET_COMPARE (&(pTimDev->pTimBaseDev->handle), TIM_ID_TO_HAL_CHAN (pTimDev->id), pulseWidth);
}

void Plat_Tim_SetCNT (TimDevice_t* pTimDev, uint32_t count) {

    if (!pTimDev || !pTimDev->pTimBaseDev) {
        return;
    }
    __HAL_TIM_SET_COUNTER (&(pTimDev->pTimBaseDev->handle), count);
}

uint32_t Plat_Tim_GetPrescaler (TimDevice_t* pTimDev) {

    if (!pTimDev || !pTimDev->pTimBaseDev) {
        return 0;
    }
    return __HAL_TIM_GET_PRESCALER (&(pTimDev->pTimBaseDev->handle));
}

uint32_t Plat_Tim_GetPeriod (TimDevice_t* pTimDev) {

    if (!pTimDev || !pTimDev->pTimBaseDev) {
        return 0;
    }
    return __HAL_TIM_GET_AUTORELOAD (&(pTimDev->pTimBaseDev->handle));
}

uint32_t Plat_Tim_GetCC (TimDevice_t* pTimDev) {

    if (!pTimDev || !pTimDev->pTimBaseDev) {
        return 0;
    }
    return __HAL_TIM_GET_COMPARE (&(pTimDev->pTimBaseDev->handle), TIM_ID_TO_HAL_CHAN (pTimDev->id));
}

uint32_t Plat_Tim_GetCNT (TimDevice_t* pTimDev) {

    if (!pTimDev || !pTimDev->pTimBaseDev) {
        return 0;
    }
    return __HAL_TIM_GET_COUNTER (&(pTimDev->pTimBaseDev->handle));
}

uint32_t Plat_Tim_GetClkFreqHz (TimDevice_t* pTimDev) {

    if (!pTimDev || !pTimDev->pTimBaseDev) {
        return 0;
    }

    uint32_t clkDiv = __HAL_TIM_GET_CLOCKDIVISION (&(pTimDev->pTimBaseDev->handle));
    uint32_t clk    = HAL_RCC_GetPCLK1Freq ();
    if (clkDiv != TIM_CLOCKDIVISION_DIV1) {
        clk *= 2;
    }
    return clk;
}

void Plat_Tim_SetInterruptEnabled (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag, bool enabled) {

    if (!pTimDev || !pTimDev->pTimBaseDev) {
        return;
    }

    if (enabled) {
        __HAL_TIM_ENABLE_IT (&(pTimDev->pTimBaseDev->handle), g_InterruptFlagsMap[flag]);
    } else {
        __HAL_TIM_DISABLE_IT (&(pTimDev->pTimBaseDev->handle), g_InterruptFlagsMap[flag]);
    }
}

void Plat_Tim_ClearInterruptFlag (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag) {

    if (!pTimDev || !pTimDev->pTimBaseDev) {
        return;
    }
    __HAL_TIM_CLEAR_FLAG (&(pTimDev->pTimBaseDev->handle), g_InterruptFlagsMap[flag]);
}

uint32_t Plat_Tim_GetInterruptFlag (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag) {

    if (!pTimDev || !pTimDev->pTimBaseDev) {
        return 0;
    }
    return __HAL_TIM_GET_FLAG (&(pTimDev->pTimBaseDev->handle), g_InterruptFlagsMap[flag]);
}

TimBaseDevice_t* Plat_Tim_InitBaseDevice (eTIM_ID_t timId, TimCfg_t* pTimCfg) {

    TimBaseDevice_t** ppTimDevHandle = TimBaseDevices_GetMutable (TIM_ID_TO_DEVICE_INDEX (timId));
    if (!ppTimDevHandle) {
        return NULL;
    }

    TimBaseDevice_t* pTimBaseDev = *ppTimDevHandle;
    if (pTimBaseDev) {
        return pTimBaseDev;
    }

    pTimBaseDev = Alloc_SharedMem (sizeof (TimBaseDevice_t));
    if (!pTimBaseDev) {
        return NULL;
    }

    TimHwCfg_t hwCfg = Plat_Tim_Get_DeviceHwCfg (timId);
    RETURN_IF_NULL (hwCfg.pRccEnableReg, NULL, "Invalid timer device ID: %u", timId);
    RETURN_IF_NULL (hwCfg.pInstance, NULL, "Invalid timer device ID: %u", timId);

    // Enable rcc clock
    *(hwCfg.pRccEnableReg) |= hwCfg.rccEnableMsk;
    DelayMicroseconds (1);
    // pTimBaseDev->id                            = TIM_ID_TO_DEVICE_ID (timId);
    pTimBaseDev->handle.Instance               = hwCfg.pInstance;
    pTimBaseDev->handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    pTimBaseDev->handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    pTimBaseDev->handle.Init.Period            = 0xFFFF;
    pTimBaseDev->handle.Init.Prescaler         = 0;
    pTimBaseDev->handle.Init.RepetitionCounter = 0;
    pTimBaseDev->handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    // deinitialize first in case it was previously initialized
    if (HAL_TIM_PWM_DeInit (&(pTimBaseDev->handle)) != HAL_OK) {
        LOG_ERROR ("Failed to de-initialize timer device handle for TIM ID: %u", timId);
        return NULL;
    }

    if (HAL_TIM_PWM_Init (&(pTimBaseDev->handle)) != HAL_OK) {
        LOG_ERROR ("Failed to initialize timer device handle for TIM ID: %u", timId);
        return NULL;
    }

    return pTimBaseDev;
}

eSTATUS_t Plat_Tim_InitCC (TimDevice_t* pTimDev) {

    if (!pTimDev || !pTimDev->pTimBaseDev) {
        return eSTATUS_NULL_ARG;
    }

    TIM_OC_InitTypeDef sConfigOC = { 0 };
    sConfigOC.OCMode             = TIM_OCMODE_PWM1;
    sConfigOC.Pulse              = 0;
    sConfigOC.OCPolarity         = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode         = TIM_OCFAST_DISABLE;
    sConfigOC.OCNPolarity        = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCNIdleState       = TIM_OCNIDLESTATE_RESET;
    sConfigOC.OCIdleState        = TIM_OCIDLESTATE_RESET;
    // clang-format off
    uint32_t halChannel = TIM_ID_TO_HAL_CHAN(pTimDev->id);
    HAL_StatusTypeDef halStatus = HAL_TIM_PWM_ConfigChannel (&(pTimDev->pTimBaseDev->handle), &sConfigOC, halChannel);
    // clang-format on

    if (halStatus != HAL_OK) {
        LOG_ERROR ("Failed to configure timer channel for timer id: %u", pTimDev->id);
        return eSTATUS_FAILURE;
    }

    HAL_NVIC_SetPriority (pTimDev->irqNum, pTimDev->irqPriority, pTimDev->irqPriority);
    HAL_NVIC_EnableIRQ (pTimDev->irqNum);

    return eSTATUS_SUCCESS;
}

eSTATUS_t Plat_Tim_Start (TimDevice_t* pTimDev, uint8_t const* pData, uint32_t size) {

    if (!pTimDev || !pTimDev->pTimBaseDev) {
        return eSTATUS_NULL_ARG;
    }
    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_TIM_PWM_Start (&(pTimDev->pTimBaseDev->handle), TIM_ID_TO_HAL_CHAN (pTimDev->id));
    RETURN_IF (status != HAL_OK, eSTATUS_FAILURE, "Failed to start timer PWM for timer");
    return eSTATUS_SUCCESS;
}

eSTATUS_t Plat_Tim_Stop (TimDevice_t* pTimDev) {

    if (!pTimDev || !pTimDev->pTimBaseDev) {
        return eSTATUS_NULL_ARG;
    }
    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_TIM_PWM_Stop (&(pTimDev->pTimBaseDev->handle), TIM_ID_TO_HAL_CHAN (pTimDev->id));
    RETURN_IF (status != HAL_OK, eSTATUS_FAILURE, "Failed to stop timer PWM for timer");
    return eSTATUS_SUCCESS;
}
