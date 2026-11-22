
#ifndef MEM_H
#define MEM_H

#include "platform.h"
#include "stdint.h"

#define MEM_U32_ALIGN4(addr) ((uint32_t)(addr) & ((uint32_t)~0x3U))

#endif // MEM_H
