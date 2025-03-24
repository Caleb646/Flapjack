#ifndef LOG_H
#define LOG_H

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"
#include "stdint.h"

#include "mem/mem.h"
#include "mem/ring_buff.h"

typedef struct {
    RingBuff volatile *pRingBuf;
    UART_HandleTypeDef *pUART;
} Log;

extern Log sLogger_;

int8_t LoggerInit(UART_HandleTypeDef *pUART);


#endif // LOG_H
