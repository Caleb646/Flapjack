#ifndef LOG_H
#define LOG_H

#include <stdint.h>
#include "stm32h7xx_hal_uart.h"

#define CM4_BUFF_ID 0
#define CM7_BUFF_ID 1

int8_t LoggerInit(UART_HandleTypeDef *pUART);


#endif // LOG_H
