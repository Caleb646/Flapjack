/**
 * \file            ringbuff.h
 * \brief           Ring buffer manager
 */

/*
 * Copyright (c) 2020 Tilen MAJERLE
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * This file is part of ring buffer library.
 *
 * Author:          Tilen MAJERLE <tilen@majerle.eu>
 * Version:         v1.3.1
 */

#ifndef RING_BUFF_H
#define RING_BUFF_H

#include <stdint.h>
#include "common.h"

/**
 * \brief           Enable buffer structure pointer parameter as volatile
 * To use this feature, uncomment keyword below
 */
#define RINGBUFF_VOLATILE                       volatile

/**
 * \brief           Adds 2 magic words to make sure if memory is corrupted
 *                  application can detect wrong data pointer and maximum size
 */
#define RINGBUFF_USE_MAGIC                      1

/**
 * \brief           Event type for buffer operations
 */
typedef enum {
    RINGBUFF_EVT_READ,                          /*!< Read event */
    RINGBUFF_EVT_WRITE,                         /*!< Write event */
    RINGBUFF_EVT_RESET,                         /*!< Reset event */
} ringbuff_evt_type_t;

/**
 * \brief           Buffer structure forward declaration
 */
typedef struct RingBuff_ RingBuff;

/**
 * \brief           Event callback function type
 * \param[in]       buff: Buffer handle for event
 * \param[in]       evt: Event type
 * \param[in]       bp: Number of bytes written or read (when used), depends on event type
 */
typedef void (*ringbuff_evt_fn)(RINGBUFF_VOLATILE RingBuff* buff, ringbuff_evt_type_t evt, size_t bp);

/**
 * \brief           Buffer structure
 */
typedef struct RingBuff_ {
    uint32_t magic1;                            
    uint8_t* buff;                              /*!< Pointer to buffer data.
                                                    Buffer is considered initialized when `buff != NULL` and `size > 0` */
    size_t size;                                /*!< Size of buffer data. Size of actual buffer is `1` byte less than value holds */
    size_t r;                                   /*!< Next read pointer. Buffer is considered empty when `r == w` and full when `w == r - 1` */
    size_t w;                                   /*!< Next write pointer. Buffer is considered empty when `r == w` and full when `w == r - 1` */
    ringbuff_evt_fn evt_fn;                     /*!< Pointer to event callback function */
    uint32_t magic2;
    // added paddding to ensure that 
    // uint8_t* buff starts at a 4 byte aligned address
    uint8_t padding_[4];
} RingBuff;

// #define xstr(s) str(s)
// #define str(s) #s
// #define err_msg(x) #x " is " xstr(x)
// #define AAA sizeof(RingBuff)

// #pragma message "content of AAA: " err_msg(AAA)

STATIC_ASSERT(sizeof(RingBuff) == 32, "RingBuff is not 32 bytes");

RINGBUFF_VOLATILE RingBuff*     RingBuffCreate(void* buffdata, size_t size);
STATUS_TYPE                     RingBuffIsValid(RINGBUFF_VOLATILE RingBuff* buff);
void                            RingBuffFree(RINGBUFF_VOLATILE RingBuff* buff);
void                            RingBuffReset(RINGBUFF_VOLATILE RingBuff* buff);
void                            RingBuffSetEvtFn(RINGBUFF_VOLATILE RingBuff* buff, ringbuff_evt_fn fn);

/* Read/Write functions */
size_t      RingBuffWrite(RINGBUFF_VOLATILE RingBuff* buff, const void* data, size_t btw);
size_t      RingBuffRead(RINGBUFF_VOLATILE RingBuff* buff, void* data, size_t btr);
size_t      RingBuffPeek(RINGBUFF_VOLATILE RingBuff* buff, size_t skip_count, void* data, size_t btp);

/* Buffer size information */
size_t      RingBuffGetFree(RINGBUFF_VOLATILE RingBuff* buff);
size_t      RingBuffGetFull(RINGBUFF_VOLATILE RingBuff* buff);

/* Read data block management */
void *      RingBuffGetLinearBlockReadAddress(RINGBUFF_VOLATILE RingBuff* buff);
size_t      RingBuffGetLinearBlockReadLength(RINGBUFF_VOLATILE RingBuff* buff);
size_t      RingBuffSkip(RINGBUFF_VOLATILE RingBuff* buff, size_t len);

/* Write data block management */
void *      RingBuffGetLinearBlockWriteAddress(RINGBUFF_VOLATILE RingBuff* buff);
size_t      RingBuffGetLinearBlockWriteLength(RINGBUFF_VOLATILE RingBuff* buff);
size_t      RingBuffAdvance(RINGBUFF_VOLATILE RingBuff* buff, size_t len);



#endif // RING_BUFF_H
