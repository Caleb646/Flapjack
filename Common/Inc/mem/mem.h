
#ifndef MEM_H
#define MEM_H

#include "mem/ring_buff.h"
#include "stdint.h"

#define MEM_U32_ALIGN4(addr) ((uint32_t)(addr) & ((uint32_t)~0x3U))

#ifdef UNIT_TEST

#define SHARED_MEM_SECTION

#else
// #define SHARED_MEM_SECTION __attribute__ ((section (".shared_mem"))) volatile
#define SHARED_MEM_SECTION __attribute__ ((section (".shared_mem")))

#endif

#endif // MEM_H
