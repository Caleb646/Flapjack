#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "core/core.h"

#include "drivers/dma.h"
#include "drivers/dma_defs.h"
#include "drivers/motor.h"
#include "drivers/tim.h"
#include "drivers/tim_defs.h"

#include "drivers/io/gpio.h"
#include "drivers/io/gpio_defs.h"

#include "cfg/motor.h"

#include "platform/platform.h"

#include "targets/target.h"

#define DSHOT_MAX_PORTS       TARG_MAX_MOTORS
#define DSHOT_MIN_THROTTLE    48U
#define DSHOT_MAX_THROTTLE    2047U
#define DSHOT_DMA_GPIO_STATES 3U
#define DSHOT_NBITS           16U
#define DMA_BUFFER_SIZE_BYTES ((DSHOT_NBITS * sizeof (uint32_t)) * DSHOT_DMA_GPIO_STATES)

static TARG_SHARED_MEM_BSS_SECTION TimDevice_t* gp_Timer = { 0 }; // s// [DSHOT_MAX_PORTS]  = { 0 };
static TARG_SHARED_MEM_BSS_SECTION int8_t volatile g_FramesInFlight = { 0 }; // s// [DSHOT_MAX_PORTS] = { 0 };

static TARG_SHARED_MEM_BSS_SECTION uint8_t g_PortsInUse                   = { 0 };
static TARG_SHARED_MEM_BSS_SECTION uint8_t g_MotorToPort[TARG_MAX_MOTORS] = { 0 };
static TARG_SHARED_MEM_BSS_SECTION GPIO_t* gp_Ports[DSHOT_MAX_PORTS]      = { 0 };

static TARG_SHARED_MEM_BSS_SECTION DmaDevice_t* gp_DmaDevs[DSHOT_MAX_PORTS] = { 0 };
static TARG_SHARED_MEM_BSS_SECTION uint32_t* gp_DmaBuffers[DSHOT_MAX_PORTS] = { 0 };

static TARG_SHARED_MEM_BSS_SECTION uint8_t g_ClrBitPos[TARG_MAX_MOTORS] = { 0 };
static TARG_SHARED_MEM_BSS_SECTION uint8_t g_SetBitPos[TARG_MAX_MOTORS] = { 0 };

static void Stm32_Dshot_DmaCallback (DmaDevice_t* pDmaDevice, void* pCtx) {

    if (!pDmaDevice || !pCtx) {
        return;
    }

    Dma_SetDeviceEnabled (pDmaDevice, false);
    if (--g_FramesInFlight <= 0) {
        g_FramesInFlight = 0;
        __HAL_TIM_DISABLE (&gp_Timer->handle);
        __HAL_TIM_DISABLE_DMA (&(gp_Timer->handle), TIM_DMA_UPDATE);
    }
}

static uint16_t Stm32_DshotPreparePacket (uint16_t value) {

    uint16_t packet         = 0U;
    uint8_t dshot_telemetry = false;

    packet = ((uint32_t)value << 1U) | (dshot_telemetry ? 1U : 0U);

    // compute checksum
    uint16_t csum      = 0;
    uint16_t csum_data = packet;

    for (uint16_t i = 0; i < 3U; i++) {
        csum ^= csum_data; // xor data by nibbles
        csum_data >>= 4U;
    }

    csum &= 0xFU;
    packet = ((uint32_t)packet << 4U) | csum;

    return packet;
}

static void Stm32_Dshot_Write (float throttle, uint8_t motorIndex, uint8_t portIndex) {

    uint16_t val = (uint16_t)mapf32 (throttle, 0.0F, 1.0F, (float)DSHOT_MIN_THROTTLE, (float)DSHOT_MAX_THROTTLE);
    uint16_t packet = Stm32_DshotPreparePacket (val);

    uint32_t* pDmaBuffer   = gp_DmaBuffers[portIndex];
    uint8_t motorSetBitPos = g_SetBitPos[motorIndex];
    uint8_t motorClrBitPos = g_ClrBitPos[motorIndex];

    for (uint32_t bitIdx = 0; bitIdx < DSHOT_NBITS; ++bitIdx) {

        pDmaBuffer[bitIdx * DSHOT_DMA_GPIO_STATES + 0U] |= (1U << motorSetBitPos); // Set high
        // if bit is 0, set low earlier
        if (!(packet & 0x8000U)) {
            pDmaBuffer[bitIdx * DSHOT_DMA_GPIO_STATES + 1U] |= (1U << motorClrBitPos); // Set low for bit 0
        }
        pDmaBuffer[bitIdx * DSHOT_DMA_GPIO_STATES + 2U] |= (1U << motorClrBitPos); // Set low
        packet <<= 1U;
    }
}

static eSTATUS_t Plat_Dshot_Write (float const* pThrottles, uint32_t nThrottles) {

    if (!pThrottles || !nThrottles || nThrottles > TARG_MAX_MOTORS) {
        return eSTATUS_INVALID_ARG;
    }

    for (uint32_t portIdx = 0; portIdx < g_PortsInUse; ++portIdx) {
        // Clear previous DMA buffer
        memset ((void*)gp_DmaBuffers[portIdx], 0, DMA_BUFFER_SIZE_BYTES);
    }

    uint32_t timeout = 100U;
    while (g_FramesInFlight > 0 && timeout-- > 0U) {
        DelayMicroseconds (10);
    }

    if (timeout == 0U) {
        return eSTATUS_FAIL;
    }

    for (uint32_t motorIdx = 0; motorIdx < nThrottles; ++motorIdx) {
        Stm32_Dshot_Write (pThrottles[motorIdx], (uint8_t)motorIdx, g_MotorToPort[motorIdx]);
    }

    for (uint32_t portIdx = 0; portIdx < g_PortsInUse; ++portIdx) {
        GPIO_TypeDef* pPort = GPIO_GetPort (gp_Ports[portIdx]);
        Dma_StartTransfer (gp_DmaDevs[portIdx], gp_DmaBuffers[portIdx], &pPort->BSRR, DMA_BUFFER_SIZE_BYTES);
    }

    g_FramesInFlight = (int8_t)g_PortsInUse;
    __HAL_TIM_ENABLE_DMA (&gp_Timer->handle, TIM_DMA_UPDATE);
    __HAL_TIM_ENABLE (&gp_Timer->handle);

    return eSTATUS_OK;
}

eSTATUS_t Plat_Dshot_Init (MotorsCfg_t const* pCfg, MotorProtVtbl_t* pOutVtbl) {

    if (!pCfg || !pOutVtbl) {
        return eSTATUS_NULL_ARG;
    }

    uint32_t portsSeen = 0U;
    for (uint32_t i = 0; i < TARG_MAX_MOTORS; ++i) {

        eGPIO_ID_t gpioId = pCfg->gpios[i];
        if (gpioId == eGPIO_ID_NULL) {
            continue;
        }

        uint32_t portMsk = (1U << GPIO_GetPortIndex (gpioId));
        if (!(portsSeen & portMsk)) {
            // TODO: set gpio owner
            GPIO_t* pGpio          = GPIO_Init (gpioId, 1U, PLAT_GPIO_CFG_OUT_PP_NOPULL, 0U);
            gp_Ports[g_PortsInUse] = GPIO_GetIO (gpioId);
            g_MotorToPort[i]       = g_PortsInUse;

            portsSeen |= portMsk;
            ++g_PortsInUse;
        }

        g_ClrBitPos[i] = (uint8_t)(GPIO_GetPinIndex (gpioId));
        g_SetBitPos[i] = (uint8_t)((GPIO_GetPinIndex (gpioId) + PLAT_GPIO_MAX_PINS));
    }

    gp_Timer = TimDev_GetByCaps (true);
    if (TimDev_InitBase (gp_Timer) != eSTATUS_OK) {
        return eSTATUS_FAIL;
    }
    TimDev_SetBestClkAndPeriod (gp_Timer, 434782U); // 434,782 Hz update rate for 2.3us bit time for DSHOT150

    for (uint32_t i = 0; i < g_PortsInUse; ++i) {

        GPIO_SetLow (gp_Ports[i]);

        gp_DmaBuffers[i] = Alloc_SharedMem (DMA_BUFFER_SIZE_BYTES);
        if (!gp_DmaBuffers[i]) {
            return eSTATUS_FAIL;
        }

        gp_DmaDevs[i] = Dma_GetFreeDevice ();
        if (Dma_InitForMemToGpio (TimDev_GetId (gp_Timer), gp_DmaDevs[i]) != eSTATUS_OK) {
            return eSTATUS_FAIL;
        }
        Dma_RegisterCallback (gp_DmaDevs[i], Stm32_Dshot_DmaCallback, NULL);
    }

    pOutVtbl->fnUpdateMotors = Plat_Dshot_Write;
    return eSTATUS_OK;
}