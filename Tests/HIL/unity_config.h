#ifndef UNITY_CONFIG_H
#define UNITY_CONFIG_H

void UART_PutChar (char c);

#define UNITY_OUTPUT_CHAR(c) UART_PutChar (c)

#endif // UNITY_CONFIG_H