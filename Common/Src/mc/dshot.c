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

#include "mc/dshot.h"
#include "common.h"
#include "conf/conf.h"
#include "dma.h"
#include "hal.h"
#include "log/logger.h"
#include "periphs/gpio.h"
#include "periphs/timer.h"
#include <stdint.h>


// 64 MHz
#define TIMER_CLOCK SystemCoreClock
#define DSHOT600_HZ PWM_MHZ2HZ (12U)
#define DSHOT300_HZ PWM_MHZ2HZ (6U)
#define DSHOT150_HZ PWM_MHZ2HZ (3U)

static DShot_t gDShotHandles[eMOTOR_ID_MAX] = { 0 };

static void DShotDMACompleteCallback (eTIMER_ID_t timerId);
static uint16_t DShotPreparePacket (uint16_t value);
static void DShotPrepareDMABuffer (DShot_t* pDShotHandle, uint16_t value);
static void DShotDMAStart (DShot_t* pDShotHandle);

static void DShotDMACompleteCallback (eTIMER_ID_t timerId) {
}

static uint16_t DShotPreparePacket (uint16_t value) {

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
static void DShotPrepareDMABuffer (DShot_t* pDShot, uint16_t value) {

    uint32_t* motor_dmabuffer   = pDShot->pMotorDmaBuffer;
    uint16_t packet             = DShotPreparePacket (value);
    uint16_t timerTicksforBit_1 = pDShot->timerTicksforBit_1;
    uint16_t timerTicksforBit_0 = pDShot->timerTicksforBit_0;

    for (uint16_t i = 0; i < 16U; ++i) {
        motor_dmabuffer[i] = (packet & 0x8000U) ? timerTicksforBit_1 : timerTicksforBit_0;
        packet <<= 1U;
    }

    motor_dmabuffer[16] = 0;
    motor_dmabuffer[17] = 0;
}


static void DShotDMAStart (DShot_t* pDShotHandle) {

    if (TimerStart (pDShotHandle->conf.timerId, pDShotHandle->pMotorDmaBuffer, DSHOT_DMA_BUFFER_SIZE) !=
        eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start DShot timer with DMA");
        return;
    }
}

static DShot_t* DShotGetById (eDEVICE_ID_t deviceId) {

    if (DEVICE_ID_IS_MOTOR (deviceId) == FALSE) {
        return NULL;
    }

    uint16_t idx = MOTOR_ID2IDX (deviceId);
    if (idx >= eMOTOR_ID_MAX) {
        return NULL;
    }

    return &gDShotHandles[idx];
}

static eSTATUS_t DShotInitBitbang (DShot_t* pDShot) {

    if (pDShot == NULL) {
        LOG_ERROR ("DShot is null");
        return eSTATUS_FAILURE;
    }

    DShotInitConf_t* pConf = &pDShot->conf;
    if (DSHOT_TYPE_IS_BITBANG (pConf->dshotType) == FALSE) {
        LOG_ERROR ("DShot type does NOT use bitbang");
        return eSTATUS_FAILURE;
    }

    eSTATUS_t status = eSTATUS_SUCCESS;
    GPIO_INIT_OUTPUT (&status, pConf->deviceId, pConf->gpioId);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize GPIO for DShot");
        return status;
    }

    pDShot->pGPIO = GPIOGetIOfromId (pConf->gpioId);

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
    switch (DSHOT_TYPE_CLEAR_TYPE (pConf->dshotType)) {
    case eDSHOT_TYPE_DMA_150:
        pDShot->timerTicksPeriod   = 427 - 1;
        pDShot->timerTicksforBit_1 = 320; // 75% of 6.67 us / 427 ticks
        pDShot->timerTicksforBit_0 = 160; // 37.5% of 6.67 us / 427 ticks

        pDShot->usPeriod      = 6.67F; // 6.67 us
        pDShot->usValforBit_1 = 5.0F;  // 5 us
        pDShot->usValforBit_0 = 2.5F;  // 2.5 us
        break;
    case eDSHOT_TYPE_DMA_300:
        // // Closet us value to 3.33 us is 3 us
        // pOutHandle->usValforBit_1 = 2; // 2 us for bit 1
        // pOutHandle->usValforBit_0 = 1; // 1 us for bit 0
        return eSTATUS_FAILURE; // DShot300 is not supported
    case eDSHOT_TYPE_DMA_600:
        // // Closet us value to 1.67 us is 2 us
        // // Closet us value to 1.25 us is 1 us
        // pOutHandle->usValforBit_1 = 1; // 1 us for bit 1
        // // Closet us value to 0.625 us is 1 us
        // pOutHandle->usValforBit_0 = 1;
        return eSTATUS_FAILURE; // DShot600 is not supported
    default:
        LOG_ERROR ("Invalid DShot type: %d", pConf->dshotType);
        return eSTATUS_FAILURE;
    }


    return eSTATUS_SUCCESS;
}

static eSTATUS_t DShotInitDMA (DShot_t* pDShot) {

    if (pDShot == NULL) {
        LOG_ERROR ("DShot is null");
        return eSTATUS_FAILURE;
    }

    DShotInitConf_t* pConf = &pDShot->conf;
    if (DSHOT_TYPE_IS_DMA (pConf->dshotType) == FALSE) {
        LOG_ERROR ("DShot type does NOT use dma");
    }

    eSTATUS_t status = eSTATUS_SUCCESS;
    TIMER_INIT_PWM_DMA (&status, pConf->deviceId, pConf->timerId);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize timer for DShot");
        return eSTATUS_FAILURE;
    }

    if (TimerRegisterCallback (pConf->timerId, DShotDMACompleteCallback, eTIMER_CALLBACK_TRANSFER_COMPLETE) !=
        eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to register timer callback for DShot");
        return eSTATUS_FAILURE;
    }

    uint16_t prescaler = 0U;
    TIMER_SET_PRESCALER (pConf->timerId, prescaler);

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
    switch (DSHOT_TYPE_CLEAR_TYPE (pConf->dshotType)) {
    case eDSHOT_TYPE_DMA_150:
        TIMER_SET_PERIOD (pConf->timerId, 427 - 1);
        pDShot->timerTicksPeriod   = 427 - 1;
        pDShot->timerTicksforBit_1 = 320; // 75% of 6.67 us / 427 ticks
        pDShot->timerTicksforBit_0 = 160; // 37.5% of 6.67 us / 427 ticks

        pDShot->usPeriod      = 6.67F; // 6.67 us
        pDShot->usValforBit_1 = 5.0F;  // 5 us
        pDShot->usValforBit_0 = 2.5F;  // 2.5 us
        break;
    case eDSHOT_TYPE_DMA_300:
        // // Closet us value to 3.33 us is 3 us
        // PWM_SET_PERIOD (ppwm, 3 - 1);
        // pOutHandle->usValforBit_1 = 2; // 2 us for bit 1
        // pOutHandle->usValforBit_0 = 1; // 1 us for bit 0
        return eSTATUS_FAILURE; // DShot300 is not supported
    case eDSHOT_TYPE_DMA_600:
        // // Closet us value to 1.67 us is 2 us
        // PWM_SET_PERIOD (ppwm, 2 - 1);
        // // Closet us value to 1.25 us is 1 us
        // pOutHandle->usValforBit_1 = 1; // 1 us for bit 1
        // // Closet us value to 0.625 us is 1 us
        // pOutHandle->usValforBit_0 = 1;
        return eSTATUS_FAILURE; // DShot600 is not supported
    default:
        LOG_ERROR ("Invalid DShot type: %d", pConf->dshotType);
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

static eSTATUS_t DShotWriteDMA (DShot_t* pDShot, uint16_t motorVal) {

    if (pDShot == NULL) {
        LOG_ERROR ("Received NULL pointer for DShot handle");
        return eSTATUS_FAILURE;
    }

    if (TimerGetChannelState (pDShot->conf.timerId) == eTIMER_CHANNEL_STATE_BUSY) {
        return eSTATUS_BUSY;
    }

    DShotPrepareDMABuffer (pDShot, motorVal);
    DShotDMAStart (pDShot);
    return eSTATUS_SUCCESS;
}

static eSTATUS_t DShotWriteBitbang (DShot_t* pDShot, uint16_t motorVal) {

    if (pDShot == NULL || pDShot->pGPIO == NULL) {
        LOG_ERROR ("Received NULL pointer for DShot handle");
        return eSTATUS_FAILURE;
    }

    float usPeriod       = pDShot->usPeriod;
    float usDelayForBit1 = pDShot->usValforBit_1;
    float usDelayForBit0 = pDShot->usValforBit_0;
    float usDelay        = 0.0F;

    ATOMIC_BLOCK_LOCAL (eNVIC_PRIO_LVL_MAX) {
        for (uint32_t i = 0; i < 16U; ++i) {

            if (motorVal & 0x8000U) {
                // bit is 1
                usDelay = (uint32_t)usDelayForBit1;
            } else {
                // bit is 0
                usDelay = (uint32_t)usDelayForBit0;
            }
            GPIO_WRITE_PIN (pDShot->pGPIO->pPort, pDShot->pGPIO->pin, GPIO_PIN_SET);
            fDelayMicroseconds (usDelay);
            GPIO_WRITE_PIN (pDShot->pGPIO->pPort, pDShot->pGPIO->pin, GPIO_PIN_RESET);
            fDelayMicroseconds (usPeriod - usDelay);
            motorVal <<= 1U;
        }
    }
}

// static eSTATUS_t
// DShotWriteMultiBitBang (eDEVICE_ID_t (deviceIds)[eMOTOR_ID_MAX], uint16_t (motorVals)[eMOTOR_ID_MAX]) {

//     ATOMIC_BLOCK_LOCAL (eNVIC_PRIO_LVL_MAX) {
//         for (uint32_t i = 0; i < 16U; ++i) {
//             for (uint32_t j = 0; j < eMOTOR_ID_MAX; ++j) {
//                 uint32_t handleIdx = MOTOR_ID2IDX (deviceIds[j]);
//                 uint32_t usDelay   = motorVals[j] & 0x8000U ?
//                                      gDShotHandles[handleIdx].usValforBit_1 :
//                                      gDShotHandles[handleIdx].usValforBit_0;
//             }
//         }
//     }
// }

eSTATUS_t DShotInit (DShotInitConf_t conf) {

    if (DEVICE_ID_IS_MOTOR (conf.deviceId) == FALSE) {
        LOG_ERROR ("deviceId is not motor");
        return eSTATUS_FAILURE;
    }

    DShot_t* pDShot = DShotGetById (conf.deviceId);
    if (pDShot == NULL) {
        LOG_ERROR ("Failed to get DShot handle by device ID");
        return eSTATUS_FAILURE;
    }

    memset (pDShot, 0, sizeof (DShot_t));
    pDShot->conf = conf;

    if (DSHOT_TYPE_IS_DMA (conf.dshotType)) {

        if (DShotInitDMA (pDShot) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize DShot DMA");
            goto error;
        }

    } else if (DSHOT_TYPE_IS_BITBANG (conf.dshotType)) {

        if (DShotInitBitbang (pDShot) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize DShot bitbang");
            goto error;
        }

    } else {
        LOG_ERROR ("Invalid DShot type: %d", conf.dshotType);
        goto error;
    }

    return eSTATUS_SUCCESS;

error:
    memset (pDShot, 0, sizeof (DShot_t));
    return eSTATUS_FAILURE;
}

eSTATUS_t DShotStart (eDEVICE_ID_t deviceId) {
    /* NOTE: Do NOT start PWM timer yet. Let it be started by DShotWrite */
    return eSTATUS_SUCCESS;
}

/*
 * Because the last sent timer CCR value is always 0, no shutdown is needed.
 * The PWM line will never go high until DShotWrite is called again.
 */
eSTATUS_t DShotStop (eDEVICE_ID_t deviceId) {
    return eSTATUS_SUCCESS;
}

eSTATUS_t DShotWrite (eDEVICE_ID_t deviceId, uint16_t motorVal) {

    DShot_t* pDShot = DShotGetById (deviceId);
    if (pDShot == NULL) {
        LOG_ERROR ("Received NULL pointer for DShot handle");
        return eSTATUS_FAILURE;
    }

    if (DSHOT_TYPE_IS_DMA (pDShot->conf.dshotType)) {
        return DShotWriteDMA (pDShot, motorVal);
    }
    if (DSHOT_TYPE_IS_BITBANG (pDShot->conf.dshotType)) {
        return DShotWriteBitbang (pDShot, motorVal);
    }

    LOG_ERROR ("Invalid DShot type: %d", pDShot->conf.dshotType);
    return eSTATUS_FAILURE;
}

// eSTATUS_t
// DShotWriteMulti (eDEVICE_ID_t (deviceIds)[eMOTOR_ID_MAX], uint16_t (motorVals)[eMOTOR_ID_MAX]) {
// }
