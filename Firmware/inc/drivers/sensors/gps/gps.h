#ifndef DEVICE_GPS_GPS_H
#define DEVICE_GPS_GPS_H

#include "core/core.h"

#include "drivers/serial/uart.h"

#include <stdbool.h>
#include <stdint.h>


typedef struct {
    float latitude;  // Degrees
    float longitude; // Degrees
    float altitude;  // Meters
    float speed;     // Meters per second
    float course;    // Degrees
    uint32_t fixAge; // Milliseconds since last fix
    uint8_t fixType; // 0 = no fix, 1 = dead reckoning, 2 = 2D fix, 3 = 3D fix
    uint8_t satellitesInUse;
} GpsData_t;

typedef struct {
    UartPort_t uartPort;
    GpsData_t data;
} Gps_t;

FJ_DECLARE_SHARED (Gps_t, g_Gps);

eSTATUS_t Gps_Init_ (Gps_t* pOutGps);
static inline eSTATUS_t Gps_Init (void) {
    return Gps_Init_ (&g_Gps);
}

eSTATUS_t Gps_Update_ (Gps_t* pGps, GpsData_t* pOutData);
static inline eSTATUS_t Gps_Update (void) {
    return Gps_Update_ (&g_Gps, &g_Gps.data);
}


#endif // DEVICE_GPS_GPS_H