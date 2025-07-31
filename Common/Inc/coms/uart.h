#ifndef COMS_UART_H
#define COMS_UART_H
#include "common.h"
#include "hal.h"

extern UART_HandleTypeDef egHandleUSART_1;

eSTATUS_t UARTSystemInit (void);

#endif // COMS_UART_H
