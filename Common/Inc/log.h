#ifndef LOG_H
#define LOG_H

#include "common.h"
#include "stm32h7xx.h"
#include <stdint.h>
#include <stdio.h>

#define LOG_(lvl, ...)               \
    do {                             \
        printf (lvl);                \
        printf (__FILE__);           \
        printf (" [%u] ", __LINE__); \
        printf (__VA_ARGS__);        \
        printf ("\r\n");             \
    } while (0)
#define LOG_INFO(...)  LOG_ ("[INFO] -- ", __VA_ARGS__)
#define LOG_DEBUG(...) LOG_ ("[DEBUG] -- ", __VA_ARGS__)
#define LOG_WARN(...)  LOG_ ("[WARN] -- ", __VA_ARGS__)
#define LOG_ERROR(...) LOG_ ("[ERROR] -- ", __VA_ARGS__)

STATUS_TYPE LoggerInit (UART_HandleTypeDef* pUART);


#endif // LOG_H
