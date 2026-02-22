#ifndef DEVICE_MAG_MAG_H
#define DEVICE_MAG_MAG_H

#include "hal.h"

#include "core/core.h"

#include "drivers/bus/spi.h"

#include "drivers/sensors/mag/mmc5983.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct Mag_s {
    SpiDev_t spiDev;
    Vec3u rawData;
    Vec3f normedData;
    uint32_t usLastUpdateTime;
    bool isInitialized;
    bool usingEXTIInterrupt;
    bool dataUpdated;
} Mag_t;

FJ_DECLARE_SHARED (Mag_t, g_Mag);

static inline Mag_t* Mag_Get (void) {
    if (!g_Mag.isInitialized) {
        return NULL;
    }
    return &g_Mag;
}

eSTATUS_t Mag_Init_ (Mag_t* pOutMag);
static inline eSTATUS_t Mag_Init (void) {
    return Mag_Init_ (&g_Mag);
}

eSTATUS_t Mag_Update_ (Mag_t* pMag, bool forcePolling, Vec3f* pOutput);
static inline eSTATUS_t Mag_Update (bool forcePolling) {
    Mag_t* pMag = Mag_Get ();
    if (!pMag) {
        return eSTATUS_FAILURE;
    }
    return Mag_Update_ (pMag, forcePolling, &pMag->normedData);
}

#endif // DEVICE_MAG_MAG_H