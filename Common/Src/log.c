#include <stdio.h>
#include <string.h>

#include "log.h"

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

Log sLogger_ = { NULL };

PUTCHAR_PROTOTYPE
{
  uint32_t cpuID = HAL_GetCurrentCPUID();
  HAL_UART_Transmit(sLogger_.pUART, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

int8_t LoggerInit(UART_HandleTypeDef *pUART)
{
  if(HAL_GetCurrentCPUID() == CM4_CPUID || pUART == NULL) return;

  memset(&sLogger_, 0, sizeof(Log));
  sLogger_.pRingBuf = RingBuffCreate((void*)MEM_SHARED_UART_RINGBUFF_START, MEM_SHARED_UART_RINGBUFF_TOTAL_LEN);
  sLogger_.pUART = pUART;

  return 0;
}