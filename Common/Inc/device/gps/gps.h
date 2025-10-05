#ifndef DEVICE_GPS_GPS_H
#define DEVICE_GPS_GPS_H

#include "common.h"
#include "conf/board.h"
#include "conf/ids.h"
#include "device/gps/gps.h"
#include "peripheral/bus/bus.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    DeviceBoardConf_t boardConf;
} GPSInitConf_t;

typedef struct {
    eBUS_ID_t busId;
    eDEVICE_ID_t deviceId;
    BusVTable_t bus;
    bool isInitialized;
} GPS_t;

typedef GPS_t vGPS_t;

eSTATUS_t GPSInit (GPSInitConf_t conf, GPS_t* pOutGPS);


#endif // DEVICE_GPS_GPS_H