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
#include "conf/board.h"
#include "conf/conf.h"
#include "conf/ids.h"
#include "core/core.h"
#include "core/log/logger.h"
#include "hal.h"
#include "mem/mem.h"
#include "peripheral/dma.h"
#include "peripheral/gpio.h"
#include "peripheral/timer.h"
#include <stdint.h>



#define DSHOT_VALID(pDSHOT) \
    (((pDSHOT) != NULL) && ((pDSHOT)->isInitialized == true) && (pDSHOT)->writeFn != NULL)

static SHARED_MEM_SECTION DShot_t gDShotHandles[MOTOR_MAX_MOTORS] = { 0 };

static void DShotDMACompleteCallback (eTIMER_ID_t timerId);
static uint16_t DShotPreparePacket (uint16_t value);
static void DShotPrepareDMABuffer (vDShot_t* pDShotHandle, uint16_t value);
static void DShotDMAStart (vDShot_t* pDShotHandle);
static eSTATUS_t DShot_InitTimings (vDShot_t* pDShot, DShotInitConf_t conf);
static eSTATUS_t DShot_BBInit (vDShot_t* pDShot, DShotInitConf_t conf);
static eSTATUS_t DShot_DMAInit (vDShot_t* pDShot, DShotInitConf_t conf);
static eSTATUS_t DShot_Write_DMA (vDShot_t* pDShot, uint16_t motorVal);
static eSTATUS_t DShot_Write_BBWithTimer (vDShot_t* pDShot, uint16_t motorVal);
static eSTATUS_t DShot_Write_BBNoTimer (vDShot_t* pDShot, uint16_t motorVal);

static void DShotDMACompleteCallback (eTIMER_ID_t timerId) {
}

static uint16_t DShotPreparePacket (uint16_t value) {

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

    if (Timer_Start (pDShot->pTimer, pDShot->pMotorDmaBuffer, DSHOT_DMA_BUFFER_SIZE) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start DShot timer with DMA");
        return;
    }
}

static eSTATUS_t DShot_InitTimings (vDShot_t* pDShot, DShotInitConf_t conf) {

    eDSHOT_TYPE_t dshotType = pDShot->dshotType;
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

        pDShot->cyclesPeriod   = US_TO_CYCLES (pDShot->usPeriod);
        pDShot->cyclesforBit_1 = US_TO_CYCLES (pDShot->usValforBit_1);
        pDShot->cyclesforBit_0 = US_TO_CYCLES (pDShot->usValforBit_0);
        break;
    default: LOG_ERROR ("Invalid DShot type"); return eSTATUS_INVALID_ARG;
    }
    return eSTATUS_SUCCESS;
}

static eSTATUS_t DShot_BBInit (vDShot_t* pDShot, DShotInitConf_t conf) {

    eSTATUS_t status                      = eSTATUS_SUCCESS;
    eDEVICE_ID_t motorId                  = conf.motorId;
    MotorDeviceConf_t motorConf           = conf.motorBoardConf;
    TimerBoardConf_t* pTimerBoardConf     = motorConf.pTimerBoardConf;
    GPIOBoardConf_t const* pGPIOBoardConf = pTimerBoardConf->pGPIOBoardConf;
    eTIMER_ID_t timerId                   = pTimerBoardConf->timerId;
    eGPIO_ID_t timerGpioId                = pTimerBoardConf->pGPIOBoardConf->id;

    uint32_t mode      = pGPIOBoardConf->conf.mode;
    uint32_t alternate = pGPIOBoardConf->conf.alternate;
    bool usingTimer    = (mode == GPIO_MODE_AF_PP);
    if (usingTimer == true) {

        TIMER_INIT_PWM (&status, motorId, timerId, 0U, *pTimerBoardConf);
        RETURN_IF (STATUS_FAIL (status), status, "Failed to initialize timer for DShot BB");

        pDShot->opMode  = eDSHOT_OP_MODE_BB_WITH_TIMER;
        pDShot->writeFn = DShot_Write_BBWithTimer;
        pDShot->pTimer  = Timer_Get_ById (timerId);
        LOG_INFO ("DShot using Timer for Bitbang");
    } else {

        // DShot Bitbang GPIO should not be in alternate function mode
        RETURN_IF (alternate != 0U, eSTATUS_INVALID_ARG, "DShot Bitbang GPIO should not be in alternate function mode");
        // Initialize GPIO in OUTPUT_PP mode
        GPIO_INIT (&status, motorId, *pGPIOBoardConf);
        RETURN_IF (STATUS_FAIL (status), eSTATUS_FAILURE, "Failed to initialize GPIO for DShot");

        pDShot->opMode  = eDSHOT_OP_MODE_BB_NO_TIMER;
        pDShot->writeFn = DShot_Write_BBNoTimer;
        pDShot->pGPIO   = GPIOGetIOfromId (timerGpioId);
        LOG_INFO ("DShot using Bitbang WITHOUT Timer");
    }

    status = DShot_InitTimings (pDShot, conf);
    RETURN_IF (STATUS_FAIL (status), status, "Failed to initialize DShot timings");

    return eSTATUS_SUCCESS;
}

static eSTATUS_t DShot_DMAInit (vDShot_t* pDShot, DShotInitConf_t conf) {

    eSTATUS_t status                  = eSTATUS_SUCCESS;
    eDEVICE_ID_t motorId              = conf.motorId;
    MotorDeviceConf_t motorConf       = conf.motorBoardConf;
    TimerBoardConf_t* pTimerBoardConf = motorConf.pTimerBoardConf;
    eTIMER_ID_t timerId               = pTimerBoardConf->timerId;

    TIMER_INIT_PWM_DMA (&status, motorId, timerId, *pTimerBoardConf);
    RETURN_IF (STATUS_FAIL (status), eSTATUS_FAILURE, "Failed to initialize timer for DShot");

    status = DShot_InitTimings (pDShot, conf);
    RETURN_IF (STATUS_FAIL (status), status, "Failed to initialize DShot timings");

    pDShot->opMode  = eDSHOT_OP_MODE_DMA;
    pDShot->writeFn = DShot_Write_DMA;
    pDShot->pTimer  = Timer_Get_ById (timerId);

    status = Timer_RegisterCallback (pDShot->pTimer, DShotDMACompleteCallback, eTIMER_CALLBACK_TRANSFER_COMPLETE);
    RETURN_IF (STATUS_FAIL (status), eSTATUS_FAILURE, "Failed to register timer callback for DShot");
    LOG_INFO ("DShot using DMA");

    return eSTATUS_SUCCESS;
}

static eSTATUS_t DShot_Write_DMA (vDShot_t* pDShot, uint16_t motorVal) {

    if (pDShot == NULL) {
        LOG_ERROR ("Received NULL pointer for DShot handle");
        return eSTATUS_FAILURE;
    }

    if (Timer_GetChannelState (pDShot->pTimer) == eTIMER_CHANNEL_STATE_BUSY) {
        return eSTATUS_BUSY;
    }

    DShotPrepareDMABuffer (pDShot, motorVal);
    DShotDMAStart (pDShot);
    return eSTATUS_SUCCESS;
}

static eSTATUS_t DShot_Write_BBWithTimer (vDShot_t* pDShot, uint16_t motorVal) {

    vTimer_t* pTimer        = pDShot->pTimer;
    uint32_t const pTicks[] = { pDShot->timerTicksforBit_0, pDShot->timerTicksforBit_1 };
    uint32_t volatile* pSR  = TIMER_GET_INT_FLAG_REG (pTimer);
    uint32_t ccrFlag        = TIMER_GET_CCIF_INT_FLAG_MASK (TIMER_GET_CHANNEL (pTimer));

    ATOMIC_BLOCK_LOCAL (eNVIC_PRIO_LVL_MAX) {
        for (uint32_t i = 0; i < 16U; ++i) {
            TIMER_SET_COMPARE (pTimer, pTicks[(motorVal & 0x8000U) > 0]);
            // Wait for the compare match event
            while ((*pSR & ccrFlag) == 0) {
            }
            *pSR &= ~ccrFlag; // Clear the compare match flag
            motorVal <<= 1U;
        }
    }

    return eSTATUS_SUCCESS;
}

static eSTATUS_t DShot_Write_BBNoTimer (vDShot_t* pDShot, uint16_t motorVal) {

    uint32_t const cyclesforBit_1 = pDShot->cyclesforBit_1 - US_TO_CYCLES (0.82F);
    uint32_t const cyclesforBit_0 = pDShot->cyclesforBit_0 - US_TO_CYCLES (0.7F);
    uint32_t const pCycles[]      = { cyclesforBit_0, cyclesforBit_1 };
    uint32_t const pCycleDiffs[] = { (pDShot->cyclesPeriod - US_TO_CYCLES (2.33F)) - cyclesforBit_0,
                                     (pDShot->cyclesPeriod - US_TO_CYCLES (2.60F)) - cyclesforBit_1 };
    GPIO_TypeDef* pPort          = pDShot->pGPIO->pPort;
    uint16_t pin                 = pDShot->pGPIO->pin;

    ATOMIC_BLOCK_LOCAL (eNVIC_PRIO_LVL_MAX) {
        for (uint32_t i = 0; i < 16U; ++i) {

            GPIO_WRITE_PIN (pPort, pin, GPIO_PIN_SET);
            DELAY_CYCLES (pCycles[(motorVal & 0x8000U) > 0]);

            GPIO_WRITE_PIN (pPort, pin, GPIO_PIN_RESET);
            DELAY_CYCLES (pCycleDiffs[(motorVal & 0x8000U) > 0]);
            motorVal <<= 1U;
        }
    }
    return eSTATUS_SUCCESS;
}

vDShot_t* DShotGetById (eDEVICE_ID_t deviceId) {

    if (DEVICE_ID_IS_MOTOR (deviceId) == false) {
        return NULL;
    }

    uint16_t idx = MOTOR_ID_TO_IDX (deviceId);
    if (idx >= MOTOR_MAX_MOTORS) {
        return NULL;
    }

    return &gDShotHandles[idx];
}

eSTATUS_t DShotInit (DShotInitConf_t conf, DShot_t* pOutDShot) {

    eDEVICE_ID_t motorId = conf.motorId;
    RETURN_IF (DEVICE_ID_IS_MOTOR (motorId) == false, eSTATUS_FAILURE, "deviceId is not motor");

    MotorDeviceConf_t motorConf = conf.motorBoardConf;

    TimerBoardConf_t* pTimerBoardConf = motorConf.pTimerBoardConf;
    RETURN_IF_NULL (pTimerBoardConf, eSTATUS_FAILURE, "Timer board config is NULL");

    GPIOBoardConf_t const* pGPIOBoardConf = pTimerBoardConf->pGPIOBoardConf;
    RETURN_IF_NULL (pGPIOBoardConf, eSTATUS_FAILURE, "Timer GPIO board config is NULL");
    bool usingDMA = motorConf.useDMA;

    vDShot_t* pDShot = DShotGetById (motorId);
    if (pOutDShot != NULL) {
        pDShot = pOutDShot;
    }
    RETURN_IF_NULL (pDShot, eSTATUS_FAILURE, "Failed to get DShot handle by device ID");
    RETURN_IF (pDShot->isInitialized, eSTATUS_FAILURE, "DShot already initialized");

    memset ((void*)pDShot, 0, sizeof (vDShot_t));
    pDShot->deviceId  = motorId;
    pDShot->dshotType = eDSHOT_TYPE_150;

    if (usingDMA == true) {
        if (DShot_DMAInit (pDShot, conf) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize DShot DMA");
            goto error;
        }

    } else {
        if (DShot_BBInit (pDShot, conf) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize DShot bitbang");
            goto error;
        }
    }

    if (pDShot->opMode == eDSHOT_OP_MODE_BB_WITH_TIMER || pDShot->opMode == eDSHOT_OP_MODE_DMA) {
        TIMER_SET_PRESCALER (pDShot->pTimer, 0U);
        TIMER_SET_PERIOD (pDShot->pTimer, pDShot->timerTicksPeriod);
    }

    LOG_INFO ("DShot initialized successfully for motor ID %d", motorId);
    pDShot->isInitialized = true;
    return eSTATUS_SUCCESS;

error:
    memset ((void*)pDShot, 0, sizeof (vDShot_t));
    return eSTATUS_FAILURE;
}

eSTATUS_t DShotStart (vDShot_t* pDShot) {

    /* NOTE: Do NOT start PWM timer yet. Let it be started by DShotWrite */
    RETURN_IF (DSHOT_VALID (pDShot) == false, eSTATUS_INVALID_ARG, "Received invalid DShot handle");
    if (pDShot->opMode == eDSHOT_OP_MODE_BB_WITH_TIMER) {
        return Timer_Start (pDShot->pTimer, NULL, 0);
    }
    return eSTATUS_SUCCESS;
}

/*
 * Because the last sent timer CCR value is always 0, no shutdown is needed.
 * The PWM line will never go high until DShotWrite is called again.
 */
eSTATUS_t DShotStop (vDShot_t* pDShot) {

    RETURN_IF (DSHOT_VALID (pDShot) == false, eSTATUS_INVALID_ARG, "Received invalid DShot handle");
    if (pDShot->opMode == eDSHOT_OP_MODE_BB_WITH_TIMER) {
        return Timer_Stop (pDShot->pTimer);
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t DShotWrite (vDShot_t* pDShot, uint16_t motorVal) {

    RETURN_IF (DSHOT_VALID (pDShot) == false, eSTATUS_INVALID_ARG, "Received invalid DShot handle");
    return pDShot->writeFn (pDShot, motorVal);
}
