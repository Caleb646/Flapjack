#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include "common.h"

int32_t clipi32(int32_t v, int32_t lower, int32_t upper)
{
    if(v < lower) return lower;
    else if (v > upper) return upper;
    else return v;
}

float clipf32(float v, float lower, float upper)
{
    if(v < lower) return lower;
    else if (v > upper) return upper;
    else return v;
}

 void CriticalErrorHandler(void)
 {
   __disable_irq();
   while (1);
 }

void __assert_func(
    const char *file, int line, const char *func, const char *failedexpr
) 
{
    __BKPT(1);
    // asm volatile ("bkpt 1");
    CriticalErrorHandler();
}