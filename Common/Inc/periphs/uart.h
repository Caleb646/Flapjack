#ifndef PERIPHS_UART_H
#define PERIPHS_UART_H
#include "common.h"
#include "hal.h"

extern UART_HandleTypeDef egHandleUSART_1;

eSTATUS_t UARTSystemInit (void);
eSTATUS_t UARTSystemEnableInterrupts (void);

#endif // PERIPHS_UART_H
