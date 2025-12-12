#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "core/core.h"

#include "drivers/tim.h"
#include "drivers/tim_defs.h"

#include "drivers/io/gpio.h"
#include "drivers/io/gpio_defs.h"

#include "platform/platform.h"

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

static TARG_SHARED_MEM_DATA_SECTION TimHwCfg_t g_TimHwCfgs[] = { { .id        = eTIM_1_DEVICE_ID,
                                                                   .pInstance = TIM1,
                                                                   .pRccEnableReg = &RCC->APB2ENR,
                                                                   .rccEnableMsk = RCC_APB2ENR_TIM1EN,
                                                                   .gpioAf = GPIO_AF1_TIM1,
                                                                   .irqNum = TIM1_UP_IRQn },
                                                                 { .id        = eTIM_2_DEVICE_ID,
                                                                   .pInstance = TIM2,
                                                                   .pRccEnableReg = &RCC->APB1LENR,
                                                                   .rccEnableMsk = RCC_APB1LENR_TIM2EN,
                                                                   .gpioAf = GPIO_AF1_TIM2,
                                                                   .irqNum = TIM2_IRQn },
                                                                 { .id        = eTIM_5_DEVICE_ID,
                                                                   .pInstance = TIM5,
                                                                   .pRccEnableReg = &RCC->APB1LENR,
                                                                   .rccEnableMsk = RCC_APB1LENR_TIM5EN,
                                                                   .gpioAf = GPIO_AF2_TIM5,
                                                                   .irqNum = TIM5_IRQn },
                                                                 { .id        = eTIM_8_DEVICE_ID,
                                                                   .pInstance = TIM8,
                                                                   .pRccEnableReg = &RCC->APB2ENR,
                                                                   .rccEnableMsk = RCC_APB2ENR_TIM8EN,
                                                                   .gpioAf = GPIO_AF3_TIM8,
                                                                   .irqNum = TIM8_UP_TIM13_IRQn },
                                                                 { .id        = eTIM_12_DEVICE_ID,
                                                                   .pInstance = TIM12,
                                                                   .pRccEnableReg = &RCC->APB1LENR,
                                                                   .rccEnableMsk = RCC_APB1LENR_TIM12EN,
                                                                   .gpioAf = GPIO_AF2_TIM12,
                                                                   .irqNum = TIM8_BRK_TIM12_IRQn },
                                                                 { .id        = eTIM_13_DEVICE_ID,
                                                                   .pInstance = TIM13,
                                                                   .pRccEnableReg = &RCC->APB1LENR,
                                                                   .rccEnableMsk = RCC_APB1LENR_TIM13EN,
                                                                   .gpioAf = GPIO_AF9_TIM13,
                                                                   .irqNum = TIM8_UP_TIM13_IRQn } };

void Stm32_Tim_IRQHandler (eTIM_DEVICE_ID_t devId) {

    TimDevice_t* pTimBaseDev = TimDev_GetById (devId);
    if (!pTimBaseDev || !pTimBaseDev->handle.Instance) {
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

// TODO add gpio ids for each channel
TimHwCfg_t* Plat_TimDev_Get_HwCfg (eTIM_DEVICE_ID_t devId) {

    for (uint32_t i = 0; i < sizeof (g_TimHwCfgs) / sizeof (g_TimHwCfgs[0]); ++i) {
        if (g_TimHwCfgs[i].id == devId) {
            return &g_TimHwCfgs[i];
        }
    }
    return NULL;
}

TimDmaReqMap_t Plat_TimDev_Get_DmaReqMap (eTIM_DEVICE_ID_t timId) {

    switch (timId) {
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

void Plat_TimDev_SetPrescaler (TimDevice_t* pTimDev, uint32_t prescaler) {

    if (!pTimDev) {
        return;
    }
    __HAL_TIM_SET_PRESCALER (&(pTimDev->handle), prescaler);
}

void Plat_TimDev_SetPeriod (TimDevice_t* pTimDev, uint32_t period) {

    if (!pTimDev) {
        return;
    }
    __HAL_TIM_SET_AUTORELOAD (&(pTimDev->handle), period);
}

void Plat_TimDev_SetCC (TimChannel_t* pTimChan, uint32_t pulseWidth) {

    if (!pTimChan || !pTimChan->pTimBaseDev) {
        return;
    }
    __HAL_TIM_SET_COMPARE (&(pTimChan->pTimBaseDev->handle), TIM_ID_TO_HAL_CHAN (pTimChan->chanId), pulseWidth);
}

void Plat_TimDev_SetCNT (TimDevice_t* pTimDev, uint32_t count) {

    if (!pTimDev) {
        return;
    }
    __HAL_TIM_SET_COUNTER (&(pTimDev->handle), count);
}

uint32_t Plat_TimDev_GetPrescaler (TimDevice_t* pTimDev) {

    if (!pTimDev) {
        return 0;
    }
    return pTimDev->handle.Instance->PSC;
}

uint32_t Plat_TimDev_GetPeriod (TimDevice_t* pTimDev) {

    if (!pTimDev) {
        return 0;
    }
    return __HAL_TIM_GET_AUTORELOAD (&(pTimDev->handle));
}

uint32_t Plat_TimDev_GetCC (TimChannel_t* pTimChan) {

    if (!pTimChan || !pTimChan->pTimBaseDev) {
        return 0;
    }
    return __HAL_TIM_GET_COMPARE (&(pTimChan->pTimBaseDev->handle), TIM_ID_TO_HAL_CHAN (pTimChan->chanId));
}

uint32_t Plat_TimDev_GetCNT (TimDevice_t* pTimDev) {

    if (!pTimDev) {
        return 0;
    }
    return __HAL_TIM_GET_COUNTER (&(pTimDev->handle));
}

uint32_t Plat_TimDev_GetClkFreqHz (TimDevice_t* pTimDev) {

    if (!pTimDev) {
        return 0;
    }

    uint32_t clkDiv = __HAL_TIM_GET_CLOCKDIVISION (&(pTimDev->handle));
    uint32_t clk    = HAL_RCC_GetPCLK1Freq ();
    if (clkDiv != TIM_CLOCKDIVISION_DIV1) {
        clk *= 2;
    }
    return clk;
}

void Plat_TimDev_SetInterruptEnabled (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag, bool enabled) {

    if (!pTimDev) {
        return;
    }

    if (enabled) {
        __HAL_TIM_ENABLE_IT (&(pTimDev->handle), g_InterruptFlagsMap[flag]);
    } else {
        __HAL_TIM_DISABLE_IT (&(pTimDev->handle), g_InterruptFlagsMap[flag]);
    }
}

void Plat_TimDev_ClearInterruptFlag (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag) {

    if (!pTimDev) {
        return;
    }
    __HAL_TIM_CLEAR_FLAG (&(pTimDev->handle), g_InterruptFlagsMap[flag]);
}

uint32_t Plat_TimDev_GetInterruptFlag (TimDevice_t* pTimDev, eTIM_INTERRUPT_FLAG_t flag) {

    if (!pTimDev) {
        return 0;
    }
    return __HAL_TIM_GET_FLAG (&(pTimDev->handle), g_InterruptFlagsMap[flag]);
}

eSTATUS_t Plat_TimDev_Init (TimDevice_t* pOutTimDev) {

    if (!pOutTimDev) {
        return eSTATUS_NULL_ARG;
    }

    TimHwCfg_t* pHwCfg = Plat_TimDev_Get_HwCfg (pOutTimDev->id);
    RETURN_IF_NULL (pHwCfg->pRccEnableReg, NULL, "Invalid timer device ID: %u", pOutTimDev->id);
    RETURN_IF_NULL (pHwCfg->pInstance, NULL, "Invalid timer device ID: %u", pOutTimDev->id);

    // Enable rcc clock
    *(pHwCfg->pRccEnableReg) |= pHwCfg->rccEnableMsk;
    DelayMicroseconds (1);
    pOutTimDev->handle.Instance               = pHwCfg->pInstance;
    pOutTimDev->handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    pOutTimDev->handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    pOutTimDev->handle.Init.Period            = 0xFFFF;
    pOutTimDev->handle.Init.Prescaler         = 0;
    pOutTimDev->handle.Init.RepetitionCounter = 0;
    pOutTimDev->handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    // deinitialize first in case it was previously initialized
    if (HAL_TIM_PWM_DeInit (&(pOutTimDev->handle)) != HAL_OK) {
        LOG_ERROR ("Failed to de-initialize timer device handle for TIM ID: %u", pOutTimDev->id);
        return eSTATUS_FAILURE;
    }

    if (HAL_TIM_PWM_Init (&(pOutTimDev->handle)) != HAL_OK) {
        LOG_ERROR ("Failed to initialize timer device handle for TIM ID: %u", pOutTimDev->id);
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t Plat_TimChan_InitCC (TimChannel_t* pChannel) {

    if (!pChannel || !pChannel->pTimBaseDev) {
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
    uint32_t halChannel = TIM_ID_TO_HAL_CHAN(pChannel->devId);
    HAL_StatusTypeDef halStatus = HAL_TIM_PWM_ConfigChannel (&(pChannel->pTimBaseDev->handle), &sConfigOC, halChannel);
    // clang-format on

    if (halStatus != HAL_OK) {
        LOG_ERROR ("Failed to configure timer channel for timer id: %u", pChannel->devId);
        return eSTATUS_FAILURE;
    }

    HAL_NVIC_SetPriority (pChannel->irqNum, pChannel->irqPriority, pChannel->irqPriority);
    HAL_NVIC_EnableIRQ (pChannel->irqNum);

    return eSTATUS_SUCCESS;
}

eSTATUS_t Plat_TimChan_Start (TimChannel_t* pChannel, uint8_t const* pData, uint32_t size) {

    if (!pChannel || !pChannel->pTimBaseDev) {
        return eSTATUS_NULL_ARG;
    }
    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_TIM_PWM_Start (&(pChannel->pTimBaseDev->handle), TIM_ID_TO_HAL_CHAN (pChannel->chanId));
    RETURN_IF (status != HAL_OK, eSTATUS_FAILURE, "Failed to start timer PWM for timer");
    return eSTATUS_SUCCESS;
}

eSTATUS_t Plat_TimChan_Stop (TimChannel_t* pChannel) {

    if (!pChannel || !pChannel->pTimBaseDev) {
        return eSTATUS_NULL_ARG;
    }
    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_TIM_PWM_Stop (&(pChannel->pTimBaseDev->handle), TIM_ID_TO_HAL_CHAN (pChannel->chanId));
    RETURN_IF (status != HAL_OK, eSTATUS_FAILURE, "Failed to stop timer PWM for timer");
    return eSTATUS_SUCCESS;
}
