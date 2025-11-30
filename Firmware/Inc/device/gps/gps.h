#ifndef DEVICE_GPS_GPS_H
#define DEVICE_GPS_GPS_H

#include "conf/board.h"
#include "conf/ids.h"
#include "core/core.h"
#include "device/gps/gps.h"
#include "peripheral/bus/bus.h"
#include <stdbool.h>
#include <stdint.h>


typedef struct {
    DevDesc_t* pDevDesc;
    Bus_t* pBus;
} GPSInitConf_t;

typedef struct {
    float latitude;  // Degrees
    float longitude; // Degrees
    float altitude;  // Meters
    float speed;     // Meters per second
    float course;    // Degrees
    uint32_t fixAge; // Milliseconds since last fix
    uint8_t fixType; // 0 = no fix, 1 = dead reckoning, 2 = 2D fix, 3 = 3D fix
    uint8_t satellitesInUse;
} GPSData_t;

typedef struct {
    eDEVICE_ID_t deviceId;
    Bus_t bus;
    bool isInitialized;
} GPS_t;

typedef GPS_t vGPS_t;

eSTATUS_t GPSInit (GPSInitConf_t conf, GPS_t* pOutGPS);
eSTATUS_t GPSStart (vGPS_t* pGPS);
eSTATUS_t GPSUpdate (vGPS_t* pGPS, GPSData_t* pOutData);
vGPS_t const* GPSGetActiveDevice (void);
vGPS_t* GPSGetMutableActiveDevice (void);


#endif // DEVICE_GPS_GPS_H