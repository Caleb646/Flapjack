#ifndef NAV_NAV_H
#define NAV_NAV_H

#include "core/core.h"

#define NAV_VALID_ATTITUDE  (1U << 0)
#define NAV_VALID_POSITION  (1U << 1)
#define NAV_VALID_VELOCITY  (1U << 2)
#define NAV_VALID_BARO_ALT  (1U << 3)

eSTATUS_t Nav_Init(void);
eSTATUS_t Nav_Update(void);

#endif // NAV_NAV_H
