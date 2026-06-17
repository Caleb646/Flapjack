#ifndef CORE_CORE_H
#define CORE_CORE_H

#include "core/core_shared.h"
#include "core/ids.h"
#include "core/log/logger.h"
#include "core/sync.h"

#include <stdbool.h>
#include <stdint.h>

eSTATUS_t Core_Init (void);

void* Allocate(uint32_t size);


#endif // CORE_CORE_H