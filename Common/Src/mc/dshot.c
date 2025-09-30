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
#include "conf/board.h"
#include "conf/conf.h"
#include "conf/ids.h"
#include "hal.h"
#include "log/logger.h"
#include "mem/mem.h"
#include "peripheral/dma.h"
#include "peripheral/gpio.h"
#include "peripheral/timer.h"
#include <stdint.h>


// 64 MHz
#define TIMER_CLOCK SystemCoreClock
#define DSHOT600_HZ PWM_MHZ2HZ (12U)
#define DSHOT300_HZ PWM_MHZ2HZ (6U)
#define DSHOT150_HZ PWM_MHZ2HZ (3U)

static SHARED_MEM_SECTION DShot_t gDShotHandles[eMOTOR_ID_MAX] = { 0 };

static void DShotDMACompleteCallback (eTIMER_ID_t timerId);
static uint16_t DShotPreparePacket (uint16_t value);
static void DShotPrepareDMABuffer (vDShot_t* pDShotHandle, uint16_t value);
static void DShotDMAStart (vDShot_t* pDShotHandle);

static void DShotDMACompleteCallback (eTIMER_ID_t timerId) {
}

static uint16_t DShotPreparePacket (uint16_t value) {

    uint16_t packet;
    uint8_t dshot_telemetry = false;

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
static void DShotPrepareDMABuffer (vDShot_t* pDShot, uint16_t value) {

    uint32_t* motor_dmabuffer   = (uint32_t*)pDShot->pMotorDmaBuffer;
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


static void DShotDMAStart (vDShot_t* pDShot) {

    eTIMER_ID_t timerId = pDShot->timerId;
    if (TIMER_START (timerId, pDShot->pMotorDmaBuffer, DSHOT_DMA_BUFFER_SIZE) !=
        eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start DShot timer with DMA");
        return;
    }
}

vDShot_t* DShotGetById (eDEVICE_ID_t deviceId) {

    if (DEVICE_ID_IS_MOTOR (deviceId) == false) {
        return NULL;
    }

    uint16_t idx = MOTOR_ID2IDX (deviceId);
    if (idx >= eMOTOR_ID_MAX) {
        return NULL;
    }

    return &gDShotHandles[idx];
}

static eSTATUS_t DShotInitBitbang (vDShot_t* pDShot, DShotInitConf_t conf) {

    if (pDShot == NULL) {
        LOG_ERROR ("DShot is null");
        return eSTATUS_FAILURE;
    }

    eSTATUS_t status                  = eSTATUS_SUCCESS;
    eDEVICE_ID_t motorId              = conf.motorId;
    MotorDeviceConf_t motorConf       = conf.motorBoardConf;
    TimerBoardConf_t* pTimerBoardConf = motorConf.pTimerBoardConf;
    GPIOBoardConf_t const* pGPIOBoardConf = pTimerBoardConf->pGPIOBoardConf;
    eTIMER_ID_t timerId     = pTimerBoardConf->timerId;
    eGPIO_ID_t timerGpioId  = pTimerBoardConf->pGPIOBoardConf->id;
    eDSHOT_TYPE_t dshotType = pDShot->dshotType;

    GPIO_INIT (&status, motorId, *pGPIOBoardConf);
    RETURN_IF (STATUS_FAIL (status), eSTATUS_FAILURE, "Failed to initialize GPIO for DShot");

    pDShot->pGPIO = GPIOGetIOfromId (timerGpioId);

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
    switch (dshotType) {
    case eDSHOT_TYPE_150:
        pDShot->timerTicksPeriod   = 427 - 1;
        pDShot->timerTicksforBit_1 = 320; // 75% of 6.67 us / 427 ticks
        pDShot->timerTicksforBit_0 = 160; // 37.5% of 6.67 us / 427 ticks

        pDShot->usPeriod      = 6.67F; // 6.67 us
        pDShot->usValforBit_1 = 5.0F;  // 5 us
        pDShot->usValforBit_0 = 2.5F;  // 2.5 us
        break;
    default:
        LOG_ERROR ("Invalid DShot type: %d", dshotType);
        return eSTATUS_FAILURE;
    }


    return eSTATUS_SUCCESS;
}

static eSTATUS_t DShotInitDMA (vDShot_t* pDShot, DShotInitConf_t conf) {

    if (pDShot == NULL) {
        LOG_ERROR ("DShot is null");
        return eSTATUS_FAILURE;
    }

    eSTATUS_t status                  = eSTATUS_SUCCESS;
    eDEVICE_ID_t motorId              = conf.motorId;
    MotorDeviceConf_t motorConf       = conf.motorBoardConf;
    TimerBoardConf_t* pTimerBoardConf = motorConf.pTimerBoardConf;
    eTIMER_ID_t timerId               = pTimerBoardConf->timerId;
    eDSHOT_TYPE_t dshotType           = pDShot->dshotType;

    TIMER_INIT_PWM_DMA (&status, motorId, timerId, *pTimerBoardConf);
    RETURN_IF (STATUS_FAIL (status), eSTATUS_FAILURE, "Failed to initialize timer for DShot");

    if (TIMER_REGISTER_CALLBACK (timerId, DShotDMACompleteCallback, eTIMER_CALLBACK_TRANSFER_COMPLETE) !=
        eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to register timer callback for DShot");
        return eSTATUS_FAILURE;
    }

    uint16_t prescaler = 0U;
    TIMER_SET_PRESCALER (timerId, prescaler);
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
    switch (dshotType) {
    case eDSHOT_TYPE_150:
        TIMER_SET_PERIOD (timerId, 427 - 1);
        pDShot->timerTicksPeriod   = 427 - 1;
        pDShot->timerTicksforBit_1 = 320; // 75% of 6.67 us / 427 ticks
        pDShot->timerTicksforBit_0 = 160; // 37.5% of 6.67 us / 427 ticks

        pDShot->usPeriod      = 6.67F; // 6.67 us
        pDShot->usValforBit_1 = 5.0F;  // 5 us
        pDShot->usValforBit_0 = 2.5F;  // 2.5 us
        break;
    default:
        LOG_ERROR ("Invalid DShot type: %d", dshotType);
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

static eSTATUS_t DShotWriteDMA (vDShot_t* pDShot, uint16_t motorVal) {

    if (pDShot == NULL) {
        LOG_ERROR ("Received NULL pointer for DShot handle");
        return eSTATUS_FAILURE;
    }

    if (TIMER_GET_CHANNEL_STATE (pDShot->timerId) == eTIMER_CHANNEL_STATE_BUSY) {
        return eSTATUS_BUSY;
    }

    DShotPrepareDMABuffer (pDShot, motorVal);
    DShotDMAStart (pDShot);
    return eSTATUS_SUCCESS;
}

static eSTATUS_t DShotWriteBitbang (vDShot_t* pDShot, uint16_t motorVal) {

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
    return eSTATUS_SUCCESS;
}

eSTATUS_t DShotInit (DShotInitConf_t conf, DShot_t* pOutDShot) {

    eDEVICE_ID_t motorId = conf.motorId;
    RETURN_IF (DEVICE_ID_IS_MOTOR (motorId) == false, eSTATUS_FAILURE, "deviceId is not motor");

    MotorDeviceConf_t motorConf = conf.motorBoardConf;

    TimerBoardConf_t* pTimerBoardConf = motorConf.pTimerBoardConf;
    RETURN_IF_NULL (pTimerBoardConf, eSTATUS_FAILURE, "Timer board config is NULL");
    GPIOBoardConf_t const* pGPIOBoardConf = pTimerBoardConf->pGPIOBoardConf;
    RETURN_IF_NULL (pGPIOBoardConf, eSTATUS_FAILURE, "Timer GPIO board config is NULL");

    eTIMER_ID_t timerId = pTimerBoardConf->timerId;
    bool usingDMA       = motorConf.useDMA;

    vDShot_t* pDShot = DShotGetById (motorId);
    if (pOutDShot != NULL) {
        pDShot = pOutDShot;
    }
    RETURN_IF_NULL (pDShot, eSTATUS_FAILURE, "Failed to get DShot handle by device ID");
    RETURN_IF (pDShot->isInitialized, eSTATUS_FAILURE, "DShot already initialized");

    memset ((void*)pDShot, 0, sizeof (vDShot_t));
    pDShot->deviceId  = motorId;
    pDShot->timerId   = timerId;
    pDShot->dshotType = eDSHOT_TYPE_150;
    pDShot->usingDMA  = usingDMA;

    if (usingDMA == true) {

        if (DShotInitDMA (pDShot, conf) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize DShot DMA");
            goto error;
        }

    } else {

        if (DShotInitBitbang (pDShot, conf) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize DShot bitbang");
            goto error;
        }
    }

    pDShot->isInitialized = true;
    return eSTATUS_SUCCESS;

error:
    memset ((void*)pDShot, 0, sizeof (vDShot_t));
    return eSTATUS_FAILURE;
}

eSTATUS_t DShotStart (vDShot_t* pDShot) {
    /* NOTE: Do NOT start PWM timer yet. Let it be started by DShotWrite */
    return eSTATUS_SUCCESS;
}

/*
 * Because the last sent timer CCR value is always 0, no shutdown is needed.
 * The PWM line will never go high until DShotWrite is called again.
 */
eSTATUS_t DShotStop (vDShot_t* pDShot) {
    return eSTATUS_SUCCESS;
}

eSTATUS_t DShotWrite (vDShot_t* pDShot, uint16_t motorVal) {

    if (pDShot == NULL) {
        LOG_ERROR ("Received NULL pointer for DShot handle");
        return eSTATUS_FAILURE;
    }

    if (pDShot->usingDMA == true) {
        return DShotWriteDMA (pDShot, motorVal);
    }
    return DShotWriteBitbang (pDShot, motorVal);
}

// eSTATUS_t
// DShotWriteMulti (eDEVICE_ID_t (deviceIds)[eMOTOR_ID_MAX], uint16_t (motorVals)[eMOTOR_ID_MAX]) {
// }
