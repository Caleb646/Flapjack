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
#include "conf/conf.h"
#include "dma.h"
#include "hal.h"
#include "periphs/timer.h"

#define DSHOT_DMA_BUFFER_SIZE 18U /* resolution + frame reset (2us) */
#define DSHOT_FRAME_SIZE      16U
#define DSHOT_MIN_THROTTLE    48U
#define DSHOT_MAX_THROTTLE    2047U
#define DSHOT_RANGE           (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)

typedef uint8_t eDSHOT_TYPE_t;
enum {
    eDSHOT_TYPE_DMA_150 = 0,
    eDSHOT_TYPE_DMA_300 = 1,
    eDSHOT_TYPE_DMA_600 = 2,

    eDSHOT_TYPE_BITBANG_150 = 4,
    eDSHOT_TYPE_BITBANG_300 = 5,
    eDSHOT_TYPE_BITBANG_600 = 6,
};

#define DSHOT_TYPE_CLEAR_TYPE(type) ((type) & 0b11U)
#define DSHOT_TYPE_IS_DMA(type)     (((type) & 0b11U) == eDSHOT_TYPE_DMA_150)
#define DSHOT_TYPE_IS_BITBANG(type) \
    (((type) & 0b11U) == eDSHOT_TYPE_BITBANG_150)

typedef struct {
    eDSHOT_TYPE_t dshotType;
    eDEVICE_ID_t deviceId;
    eTIMER_ID_t timerId;
    GPIOOutput_t gpio;
} DShotInitConf_t;

typedef struct {
    DShotInitConf_t conf;
    uint32_t pMotorDmaBuffer[DSHOT_DMA_BUFFER_SIZE]; /*!< DMA buffer for DShot */

    uint16_t timerTicksPeriod;
    uint16_t timerTicksforBit_1; /*!< microsecond value to send a 1 */
    uint16_t timerTicksforBit_0; /*!< microsecond value to send a 0 */

    float usPeriod;      /*!< microsecond value for one bit period */
    float usValforBit_1; /*!< microsecond value to send a 1 */
    float usValforBit_0; /*!< microsecond value to send a 0 */
} DShot_t;


/* Functions */
eSTATUS_t DShotInit (DShotInitConf_t conf);
eSTATUS_t DShotStart (eDEVICE_ID_t deviceId);
eSTATUS_t DShotStop (eDEVICE_ID_t deviceId);
eSTATUS_t DShotWrite (eDEVICE_ID_t deviceId, uint16_t motor_value);


#endif /* __MOTION_CONTROL_DSHOT_H__ */