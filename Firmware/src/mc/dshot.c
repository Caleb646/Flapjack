#include "hal.h"
#include "target.h"

#include "mc/dshot.h"

#include "core/core.h"

#include "drivers/dma.h"
#include "drivers/io/gpio.h"
#include "drivers/timer.h"

#include <stdint.h>
#include <string.h>


FJ_DEFINE_SHARED (DShotBB_t, g_DShotBB) = {
    .hardware = { .pTimerInstance = BRD_GET_TIMER_INSTANCE (MOTOR_1),
                  .timerAf        = BRD_GET_TIMER_AF (MOTOR_1),
                  .motorPins      = { { .timerChannel = BRD_GET_TIMER_CHANNEL (MOTOR_1),
                                        .pPort        = BRD_GET_GPIO_PORT (MOTOR_1),
                                        .pin          = BRD_GET_GPIO_PIN (MOTOR_1) },
#if defined(MOTOR_2_ENABLED) && (MOTOR_2_ENABLED == 1U)
                                 { .timerChannel = BRD_GET_TIMER_CHANNEL (MOTOR_2),
                                   .pPort        = BRD_GET_GPIO_PORT (MOTOR_2),
                                   .pin          = BRD_GET_GPIO_PIN (MOTOR_2) }
#endif
                  } }
};

#ifndef UNIT_TEST
static uint16_t DShotPreparePacket (uint16_t value);
#endif

static void DShotDMA_TC_Callback (DMA_HandleTypeDef* hdma) {
    (void)hdma;
    DShotBB_t* p = &g_DShotBB;
    __HAL_TIM_DISABLE_DMA (&p->tim6Handle, TIM_DMA_UPDATE);
    __HAL_TIM_DISABLE (&p->tim6Handle);
    p->txDone = true;
}

eSTATUS_t DShotBB_Init (void) {

    DShotBB_t* p = &g_DShotBB;

    /* Configure motor GPIO pins as plain push-pull outputs */
    GPIO_InitTypeDef gpioInit = { 0 };
    gpioInit.Mode             = GPIO_MODE_OUTPUT_PP;
    gpioInit.Pull             = GPIO_NOPULL;
    gpioInit.Speed            = GPIO_SPEED_FREQ_VERY_HIGH;
    for (uint8_t i = 0; i < BRD_MOTOR_COUNT; ++i) {
        DShotBBMotorPin_t motorPin = p->hardware.motorPins[i];
        if (!motorPin.pPort) {
            continue;
        }
        GPIO_ENABLE_CLOCK (motorPin.pPort);
        gpioInit.Pin = motorPin.pin;
        HAL_GPIO_Init (motorPin.pPort, &gpioInit);
        GPIO_SetLow (motorPin.pPort, motorPin.pin);
    }

    /* TIM6 pacer: ARR=52 → 53 ticks/sample at 64 MHz */
    __HAL_RCC_TIM6_CLK_ENABLE ();
    p->tim6Handle.Instance               = TIM6;
    p->tim6Handle.Init.Prescaler         = 0;
    p->tim6Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    p->tim6Handle.Init.Period            = DSHOT_PACER_ARR;
    p->tim6Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init (&p->tim6Handle) != HAL_OK) {
        LOG_ERROR ("Failed to initialize DShot pacer timer");
        return eSTATUS_FAILURE;
    }

    /* DMA: TIM6 UPDATE event → motor GPIO port BSRR, mem→periph, word, normal */
    DmaHandle_t* pDma = DmaResource_Alloc ();
    if (!pDma) {
        LOG_ERROR ("No free DMA stream for DShot");
        return eSTATUS_FAILURE;
    }
    p->pDmaHandle                       = pDma;
    pDma->plat.Init.Request             = DMA_REQUEST_TIM6_UP;
    pDma->plat.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    pDma->plat.Init.PeriphInc           = DMA_PINC_DISABLE;
    pDma->plat.Init.MemInc              = DMA_MINC_ENABLE;
    pDma->plat.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    pDma->plat.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    pDma->plat.Init.Mode                = DMA_NORMAL;
    pDma->plat.Init.Priority            = DMA_PRIORITY_HIGH;
    pDma->plat.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (Dma_Init (pDma) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize DShot DMA");
        return eSTATUS_FAILURE;
    }
    HAL_DMA_RegisterCallback (&pDma->plat, HAL_DMA_XFER_CPLT_CB_ID, DShotDMA_TC_Callback);
    return eSTATUS_SUCCESS;
}

eSTATUS_t DShotBB_Write (uint16_t motorVals[BRD_MOTOR_COUNT]) {

    DShotBB_t* p = &g_DShotBB;

    /* Build combined set/clear masks for all motor pins (assumed same GPIO port) */
    uint32_t set_mask = 0U;
    uint32_t clr_mask = 0U;
    for (uint8_t m = 0; m < BRD_MOTOR_COUNT; ++m) {
        set_mask |= p->hardware.motorPins[m].pin;
        clr_mask |= (p->hardware.motorPins[m].pin << 16U);
    }

    memset (p->buffer, 0, sizeof (p->buffer));

    for (uint8_t m = 0; m < BRD_MOTOR_COUNT; ++m) {
        uint16_t packet = DShotPreparePacket (motorVals[m]);
        for (uint8_t bit = 0; bit < DSHOT_FRAME_SIZE; ++bit) {
            uint8_t hi    = (packet & 0x8000U) ? DSHOT_SAMPLES_FOR_1 : DSHOT_SAMPLES_FOR_0;
            uint32_t base = (uint32_t)bit * DSHOT_SAMPLES_PER_BIT;
            for (uint8_t s = 0; s < DSHOT_SAMPLES_PER_BIT; ++s) {
                p->buffer[base + s] |= (s < hi) ? set_mask : clr_mask;
            }
            packet <<= 1U;
        }
    }
    /* Reset slots (2 bits = 16 samples): keep pins LOW */
    for (uint32_t s = DSHOT_FRAME_SIZE * DSHOT_SAMPLES_PER_BIT; s < DSHOT_BUFFER_SIZE; ++s) {
        p->buffer[s] = clr_mask;
    }

    p->txDone = false;

    /* Write first sample now; DMA handles entries 1..DSHOT_BUFFER_SIZE-1 */
    // p->hardware.motorPins[0].pPort->BSRR = p->buffer[0];

    DMA_HandleTypeDef* pDma = &p->pDmaHandle->plat;
    HAL_DMA_Start_IT (pDma, (uint32_t)&p->buffer[0], (uint32_t)&p->hardware.motorPins[0].pPort->BSRR, DSHOT_BUFFER_SIZE);
    __HAL_TIM_CLEAR_FLAG (&p->tim6Handle, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_DMA (&p->tim6Handle, TIM_DMA_UPDATE);
    __HAL_TIM_SET_COUNTER (&p->tim6Handle, 0U);
    __HAL_TIM_ENABLE (&p->tim6Handle);
    return eSTATUS_SUCCESS;
}


STATIC uint16_t DShotPreparePacket (uint16_t value) {

    uint16_t packet         = 0U;
    uint8_t dshot_telemetry = false;

    packet = ((uint32_t)value << 1U) | (dshot_telemetry ? 1U : 0U);

    /* compute checksum */
    uint16_t csum      = 0;
    uint16_t csum_data = packet;

    for (uint16_t i = 0; i < 3U; i++) {
        csum ^= csum_data; /* xor data by nibbles */
        csum_data >>= 4U;
    }

    csum &= 0xFU;
    packet = ((uint32_t)packet << 4U) | csum;

    return packet;
}
