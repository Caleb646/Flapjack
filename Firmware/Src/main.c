
#include "cmsis_os.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"


int main (void) {

    HAL_Init ();
    SPI_HandleTypeDef hspi1;
    HAL_SPI_Init (&hspi1);
    for (;;)
        ;
    return 0;
}