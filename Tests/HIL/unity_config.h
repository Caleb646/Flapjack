#ifndef UNITY_CONFIG_H
#define UNITY_CONFIG_H

void Uart_PutChar (void* p, char c);

#define UNITY_OUTPUT_CHAR(c) Uart_PutChar (NULL, c)

#endif // UNITY_CONFIG_H