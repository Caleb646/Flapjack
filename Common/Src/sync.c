#include "stm32h7xx_it.h"

/*
* \brief A SEV instruction was executed by CM7 
* and the SEV IRQ handler for CM4 was called.
* This function was originally defined in CM4/Core/Src/stm32h7xx_it.c 
* by the code generator but I moved it here.
*/
void CM7_SEV_IRQHandler(void)
{

}

/*
* \brief A sev instruction was executed by CM4 
* and the SEV IRQ handler for CM7 was called.
* This function was originally defined in CM7/Core/Src/stm32h7xx_it.c 
* by the code generator but I moved it here.
*/
void CM4_SEV_IRQHandler(void)
{

}