#ifndef PERIPHS_UART_H
#define PERIPHS_UART_H
#include "common.h"
#include "hal.h"

extern UART_HandleTypeDef egHandleUSART_1;

eSTATUS_t UARTSystemInit (void);

#endif // PERIPHS_UART_H
