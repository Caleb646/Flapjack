#ifndef TARGET_H
#define TARGET_H

#define TARG_MAX_MOTORS         2
#define TARG_MAX_SERVOS         8
#define TARG_MAX_IMUS           1
#define TARG_MAX_BAROMETERS     1
#define TARG_MAX_GPS            1

#define TARG_SHARED_MEM_SECTION __attribute__ ((section (".shared_mem")))

// include target board last to allow overrides
#include "targets/stm32h747i_disco.h"

#endif