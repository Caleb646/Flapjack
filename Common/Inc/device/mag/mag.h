#ifndef MAG_MAG_H
#define MAG_MAG_H

#include "common.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "device/mag/mmc5983.h"
#include "hal.h"
#include <stdint.h>
#include <string.h>


typedef struct {
    DeviceBoardConf_t boardConf;
} MagInitConf_t;

typedef struct {
    eDEVICE_ID_t deviceId;
    eBUS_ID_t busId;
} Mag_t;

inline eSTATUS_t MagInit (MagInitConf_t conf);
inline eSTATUS_t MagStart ();
inline eSTATUS_t MagStop ();
inline eSTATUS_t MagProcessUpdateFromINT (Vec3f* pOutput);
inline eSTATUS_t MagProcessUpdateFromPolling (Vec3f* pOutput);

#endif // MAG_MAG_H