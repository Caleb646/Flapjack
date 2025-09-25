#ifndef MAG_MAG_H
#define MAG_MAG_H

#include "common.h"
#include "hal.h"
#include "mmc5983.h"
#include <stdint.h>


typedef struct {

} MagInitConf_t;

typedef struct {

} MagDev_t;

inline eSTATUS_t MagInit (MagInitConf_t const* pMagInitConf);
inline eSTATUS_t MagStart ();
inline eSTATUS_t MagStop ();
inline eSTATUS_t MagProcessUpdateFromINT (Vec3f* pOutput);
inline eSTATUS_t MagProcessUpdateFromPolling (Vec3f* pOutput);

#endif // MAG_MAG_H