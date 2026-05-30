#ifndef MISSION_MISSION_H
#define MISSION_MISSION_H

#include "core/core.h"

typedef uint8_t eMissionMode_t;
enum {
    eMISSION_MODE_MANUAL        = 0,
    eMISSION_MODE_ALTITUDE_HOLD = 1,
    eMISSION_MODE_POSITION_HOLD = 2,
    eMISSION_MODE_WAYPOINT      = 3,
    eMISSION_MODE_RTL           = 4,
};

eSTATUS_t Mission_Init(void);
eSTATUS_t Mission_Update(void);

#endif // MISSION_MISSION_H
