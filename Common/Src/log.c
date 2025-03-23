#include "log.h"
#include <stdio.h>
#include <string.h>


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

#define BUFFER_SIZE 256
uint8_t pM7Buf[BUFFER_SIZE];
// TODO: make sure m4 buffer is reachable by the m4 and m7 core
uint8_t pM4Buf[BUFFER_SIZE];
uint8_t *pM7Read = pM7Buf, *pM7Write = pM7Buf;
uint8_t *pM4Read = pM4Buf, *pM4Write = pM4Buf;

Log sLogger_ = {NULL};

PUTCHAR_PROTOTYPE
{
  uint32_t cpuID = HAL_GetCurrentCPUID();
  uint8_t *read, *write;
  HAL_UART_Transmit(sLogger_.pUART, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

int8_t LoggerInit(UART_HandleTypeDef *pUART)
{
    if(HAL_GetCurrentCPUID() == CM4_CPUID) return;

    memset(&sLogger_, 0, sizeof(Log));
    sLogger_.pUART = pUART;

    return 0;
}