#ifndef LOG_H
#define LOG_H

#include <stdio.h>
#include <stdint.h>
#include "stm32h7xx_hal_uart.h"

#define LOG_INFO(...)          \
    do {                       \
        printf ("[INFO] -- "); \
        printf (__VA_ARGS__);  \
        printf ("\r\n");       \
    } while (0)
#define LOG_DEBUG(...)          \
    do {                        \
        printf ("[DEBUG] -- "); \
        printf (__VA_ARGS__);   \
        printf ("\r\n");        \
    } while (0)
#define LOG_WARN(...)          \
    do {                       \
        printf ("[WARN] -- "); \
        printf (__VA_ARGS__);  \
        printf ("\r\n");       \
    } while (0)
#define LOG_ERROR(...)          \
    do {                        \
        printf ("[ERROR] -- "); \
        printf (__VA_ARGS__);   \
        printf ("\r\n");        \
    } while (0)

int8_t LoggerInit(UART_HandleTypeDef *pUART);


#endif // LOG_H
