/*
* MIT License

* Copyright (c) 2023 Eunhye Seok
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

/*
 * dshot.c
 *
 *  Created on: 2021. 1. 27.
 *      Author: mokhwasomssi
 */

#include "motion_control/dshot.h"
#include "common.h"
#include "dma.h"
#include "hal.h"
#include "log.h"
#include "motion_control/pwm.h"
#include <stdint.h>

// in Hz, typically 64 MHz for STM32H7 series
#define TIMER_CLOCK SystemCoreClock

#define DSHOT600_HZ PWM_MHZ2HZ (12)
#define DSHOT300_HZ PWM_MHZ2HZ (6)
#define DSHOT150_HZ PWM_MHZ2HZ (3)

static void DShotDMACompleteCallback (TIM_HandleTypeDef* htim);
static uint32_t dshot_tim_channel_to_dma_ccx_id (uint32_t channelID);
static uint16_t dshot_prepare_packet (uint16_t value);
static void dshot_prepare_dmabuffer (DShotHandle* pDShotHandle, uint16_t value);
static void dshot_dma_start (DShotHandle* pDShotHandle);


STATUS_TYPE
DShotInit (DShotConfig dConfig, PWMConfig timConfig, DMAConfig dmaConfig, DShotHandle* pOutHandle) {

    if (pOutHandle == NULL) {
        LOG_ERROR ("Received invalid DShot configuration pointer or output handle pointer");
        return eSTATUS_FAILURE;
    }

    memset (pOutHandle, 0, sizeof (DShotHandle));
    // DShotHandle dshot = { 0 };
    uint32_t channelID    = timConfig.base.channelID;
    uint32_t timDmaRegIdx = dshot_tim_channel_to_dma_ccx_id (channelID);
    PWM_DMAConfig pwm_DmaConfig = { .base = timConfig.base };
    /*
     * Set the timer channel capture compare register to be mapped to the dma register index
     */
    pwm_DmaConfig.dmaRegIDXs[0]  = timDmaRegIdx;
    pwm_DmaConfig.dmaRegIDXCount = 1;

    STATUS_TYPE status = PWM_DMAInit (pwm_DmaConfig, dmaConfig, &pOutHandle->pwmdma);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize PWM DMA for DShot");
        return eSTATUS_FAILURE;
    }

    status =
    PWM_DMARegisterCallback (&pOutHandle->pwmdma, channelID, DShotDMACompleteCallback, ePWM_DMA_CB_TRANSFER_COMPLETE);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to register PWM DMA callback for DShot");
        return eSTATUS_FAILURE;
    }

    /*
     * Scale clock frequency from 64 MHz to 1 MHz.
     * Then set ARR to the us value in the Bit column.
     * https://brushlesswhoop.com/dshot-and-bidirectional-dshot/
     * DSHOT	    Bitrate	    T1H	    T0H	    Bit (µs)	Frame (µs)
     *    150	150kbit/s	5.00	2.50	6.67	    106.72
     *    300	300kbit/s	2.50	1.25	3.33	    53.28
     *    600	600kbit/s	1.25	0.625	1.67	    26.72
     *
     * T1H is the duration in µs for which the signal needs to be high in
     * order to be counted as a 1. T0H is the duration in µs for which the
     * signal needs to be high in order to be counted as a 0. Bit is pwm
     * period in us
     */
    // PWM_SET_PRESCALER (&dshot.pwmdma.pwm, (TIMER_CLOCK / 64) - 1U);
    // PWM_SET_PERIOD (&dshot.pwmdma.pwm, 1000); // Set period to 1 us
    // dshot.usValforBit_1 = 750;
    // dshot.usValforBit_0 = 250;
    uint16_t prescaler = (TIMER_CLOCK / 1000000U) - 1U;
    PWMHandle* ppwm    = &pOutHandle->pwmdma.pwm;
    PWM_SET_PRESCALER (ppwm, prescaler);

    DSHOTTYPE dshotType = dConfig.dshotType;
    if (dshotType == DSHOT150) {
        // Closet us value to 6.67 us is 7 us
        PWM_SET_PERIOD (ppwm, 7 - 1);
        pOutHandle->usValforBit_1 = 5; // 5 us for bit 1
        pOutHandle->usValforBit_0 = 3; // 3 us for bit 0
    }

    else if (dshotType == DSHOT300) {
        // Closet us value to 3.33 us is 3 us
        PWM_SET_PERIOD (ppwm, 3 - 1);
        pOutHandle->usValforBit_1 = 2; // 2 us for bit 1
        pOutHandle->usValforBit_0 = 1; // 1 us for bit 0
    }

    else if (dshotType == DSHOT600) {
        // Closet us value to 1.67 us is 2 us
        PWM_SET_PERIOD (ppwm, 2 - 1);
        // Closet us value to 1.25 us is 1 us
        pOutHandle->usValforBit_1 = 1; // 1 us for bit 1
        // Closet us value to 0.625 us is 1 us
        pOutHandle->usValforBit_0 = 1;
    }

    else {
        LOG_ERROR ("Invalid DShot type: %d", dshotType);
        return eSTATUS_FAILURE;
    }

    // pOutHandle->timDmaCCXRegIdx = timDmaRegIdx;
    return eSTATUS_SUCCESS;
}

STATUS_TYPE DShotStart (DShotHandle* pDShotHandle) {
    // Start the timer channel now.
    // Enabling/disabling DMA request can restart a new cycle without PWM
    // start/stop. return PWM_DMAStart (&pDShotHandle->pwmdma);

    /* NOTE: Do NOT start PWM timer yet. Let it be started by DShotWrite */
    return eSTATUS_SUCCESS;
}

STATUS_TYPE DShotWrite (DShotHandle* pDShotHandle, uint16_t motorVal) {
    if (pDShotHandle == NULL) {
        LOG_ERROR ("Received NULL pointer for DShot handle");
        return eSTATUS_FAILURE;
    }

    HAL_TIM_ChannelStateTypeDef channelState = TIM_CHANNEL_STATE_GET (
    &pDShotHandle->pwmdma.pwm.timer, pDShotHandle->pwmdma.pwm.channelID);
    if (channelState == HAL_TIM_CHANNEL_STATE_BUSY) {
        LOG_ERROR ("Tim busy");
        return eSTATUS_FAILURE;
    }
    LOG_INFO ("Tim not busy");
    dshot_prepare_dmabuffer (pDShotHandle, motorVal);
    dshot_dma_start (pDShotHandle);
    return eSTATUS_SUCCESS;
}

static void DShotDMACompleteCallback (TIM_HandleTypeDef* htim) {

    // LOG_INFO ("DShot DMA transfer complete callback");
    if (htim == NULL) {
        return;
    }

    uint32_t channel = 0;
    uint32_t dma_id  = 0;

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        channel = TIM_CHANNEL_1;
        dma_id  = TIM_DMA_ID_CC1;
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        channel = TIM_CHANNEL_2;
        dma_id  = TIM_DMA_ID_CC2;
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
        channel = TIM_CHANNEL_3;
        dma_id  = TIM_DMA_ID_CC3;
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
        channel = TIM_CHANNEL_4;
        dma_id  = TIM_DMA_ID_CC4;
    } else {
        return;
    }

    // Disable DMA request for this channel but keep timer running
    // __HAL_TIM_DISABLE_DMA (htim, dma_id);
    // __HAL_DMA_DISABLE (htim->hdma[dma_id]);
    // // Set channel state to ready for next transmission
    // TIM_CHANNEL_STATE_SET (htim, channel, HAL_TIM_CHANNEL_STATE_READY);

    if (htim->hdma[dma_id]->State == HAL_DMA_STATE_BUSY) {
        HAL_TIM_PWM_Stop_DMA (htim, channel);
    } else {
        HAL_TIM_PWM_Stop (htim, channel);
    }

    // HAL_TIM_PWM_Stop_DMA (htim, channel);
}

static uint32_t dshot_tim_channel_to_dma_ccx_id (uint32_t channelID) {
    switch (channelID) {
    case TIM_CHANNEL_1: return eTIM_DMA_ID_CC1;
    case TIM_CHANNEL_2: return eTIM_DMA_ID_CC2;
    case TIM_CHANNEL_3: return eTIM_DMA_ID_CC3;
    case TIM_CHANNEL_4: return eTIM_DMA_ID_CC4;
    default:
        LOG_ERROR ("Invalid timer channel ID: 0x%X", (uint16_t)channelID);
        return 0; // Invalid channel ID
    }
}

static uint16_t dshot_prepare_packet (uint16_t value) {

    uint16_t packet;
    uint8_t dshot_telemetry = FALSE;

    packet = (value << 1U) | (dshot_telemetry ? 1U : 0U);

    // compute checksum
    uint16_t csum      = 0;
    uint16_t csum_data = packet;

    for (uint16_t i = 0; i < 3U; i++) {
        csum ^= csum_data; // xor data by nibbles
        csum_data >>= 4U;
    }

    csum &= 0xFU;
    packet = (packet << 4U) | csum;

    return packet;
}

// Convert 16 bits packet to 16 pwm signal
static void dshot_prepare_dmabuffer (DShotHandle* pDShotHandle, uint16_t value) {

    uint32_t* motor_dmabuffer = pDShotHandle->pMotorDmaBuffer;
    uint16_t packet           = dshot_prepare_packet (value);
    uint16_t usValforBit_1    = pDShotHandle->usValforBit_1;
    uint16_t usValforBit_0    = pDShotHandle->usValforBit_0;

    for (uint16_t i = 0; i < 16U; ++i) {
        motor_dmabuffer[i] = (packet & 0x8000U) ? usValforBit_1 : usValforBit_0;
        packet <<= 1U;
    }

    // for (uint16_t i = 0; i < 16U; ++i) {
    //     motor_dmabuffer[i] = (i & 1U) ? usValforBit_1 : usValforBit_0;
    //     packet <<= 1U;
    // }

    motor_dmabuffer[16] = 0;
    motor_dmabuffer[17] = 0;
}


static void dshot_dma_start (DShotHandle* pDShotHandle) {

    if (PWM_DMAStart (&pDShotHandle->pwmdma, pDShotHandle->pMotorDmaBuffer, DSHOT_DMA_BUFFER_SIZE) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start DShot DMA");
        return;
    }
}
