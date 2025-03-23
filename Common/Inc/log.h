#ifndef LOG_H
#define LOG_H

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"
#include "stdint.h"

typedef struct {
    UART_HandleTypeDef *pUART;
} Log;

extern Log sLogger_;

int8_t LoggerInit(UART_HandleTypeDef *pUART);


#endif // LOG_H
