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
 * dshot.h
 *
 *  Created on: 2021. 1. 27.
 *      Author: mokhwasomssi
 */


#ifndef __MOTION_CONTROL_DSHOT_H__
#define __MOTION_CONTROL_DSHOT_H__

#include "common.h"
#include "dma.h"
#include "hal.h"
#include "motion_control/pwm.h"

#define DSHOT_DMA_BUFFER_SIZE 18 /* resolution + frame reset (2us) */
#define DSHOT_FRAME_SIZE      16
#define DSHOT_MIN_THROTTLE    48
#define DSHOT_MAX_THROTTLE    2047
#define DSHOT_RANGE           (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)

/* Enumeration */
typedef enum { DSHOT150, DSHOT300, DSHOT600 } DSHOTTYPE;

typedef struct {
    DSHOTTYPE dshotType; /*!< DShot type */
    // TIM_TypeDef* pTimer;  /*!< Pointer to the timer instance */
    // uint32_t timChannelID;
} DShotConfig;

typedef struct {
    PWM_DMAHandle pwm;
    eTIM_DMA_REGIDX timDmaCCXRegIdx; /*!< DMA register index for the timer channel */
    uint32_t pMotorDmaBuffer[DSHOT_DMA_BUFFER_SIZE]; /*!< DMA buffer for DShot */
    uint16_t usValforBit_1; /*!< microsecond value to send a 1 */
    uint16_t usValforBit_0; /*!< microsecond value to send a 0 */
} DShotHandle;


/* Functions */
STATUS_TYPE
DShotInit (DShotConfig* pConfig, PWMConfig* pTimConfig, DShotHandle* pOutHandle);
STATUS_TYPE DShotStart (DShotHandle* pDShotHandle);
STATUS_TYPE DShotWrite (DShotHandle* pDShotHandle, uint16_t motor_value);


#endif /* __MOTION_CONTROL_DSHOT_H__ */