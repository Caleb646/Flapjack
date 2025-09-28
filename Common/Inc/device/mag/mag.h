#ifndef DEVICE_MAG_MAG_H
#define DEVICE_MAG_MAG_H

#include "common.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "conf/ids.h"
#include "device/mag/mmc5983.h"
#include "hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef int8_t eMAG_STATUS_t;
enum {
    eMAG_I2C_INITIALIZATION_ERROR = eSTATUS_SUB_STATUS_START,
    eMAG_SPI_INITIALIZATION_ERROR,
    eMAG_INVALID_DEVICE,
    eMAG_BUS_ERROR,
    eMAG_INVALID_FILTER_BANDWIDTH,
    eMAG_INVALID_CONTINUOUS_FREQUENCY,
    eMAG_INVALID_PERIODIC_SAMPLES,
    eMAG_DATA_NOT_READY,
};

typedef struct {
    DeviceBoardConf_t boardConf;
} MagInitConf_t;

typedef struct Mag_s {
    eDEVICE_ID_t deviceId;
    eBUS_ID_t busId;
    uint8_t nSPIDummyBytes;
    Vec3u rawData;
    uint32_t msLastUpdateTime;
    bool isInitialized;
    bool usingInterrupt;
    bool usingDMA;
    bool dataUpdated;
} Mag_t;

typedef Mag_t vMag_t;

eMAG_STATUS_t MagInit (MagInitConf_t conf);
eMAG_STATUS_t MagStart (vMag_t* pMag);
eMAG_STATUS_t MagStop (vMag_t* pMag);
eMAG_STATUS_t MagUpdate (vMag_t* pMag, bool forcePolling, Vec3f* pOutput);
vMag_t* MagGetActiveDevice (void);

#endif // DEVICE_MAG_MAG_H