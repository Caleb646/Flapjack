#ifndef DRIVERS_GPS_GPS_DRIVER_H
#define DRIVERS_GPS_GPS_DRIVER_H

/*
 * Generic, device-agnostic GPS driver interface.
 *
 * The application layer (devices/gps.c) talks to the receiver only through the
 * GpsDriver_t vtable below. `Read` returns a decoded fix in engineering units -
 * sentence assembly, checksum validation and the NMEA (or UBX) parse are backend
 * concerns.
 *
 * `Read` returns SUCCESS only for a sentence that actually carried a position.
 * A receiver with no lock (void RMC, GGA with fix_quality 0) and the
 * non-positional sentence types return eSTATUS_UNSUPPORTED, so a caller that
 * publishes on SUCCESS cannot mistake "no fix" for a fix at 0,0.
 *
 * This header is the boundary: it must not depend on any backend header.
 */

#include "core/core.h"

#include "drivers/device.h"
#include "drivers/serial/uart.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    /* Degrees, double rather than float: at 37 deg latitude a binary32 degree
     * quantises to ~0.4 m, and umsg_sensors_gps_t carries doubles anyway - there
     * is no reason to narrow and then widen again. */
    double latitude;
    double longitude;
    float altitude;  // Meters
    float speed;     // Meters per second
    float course;    // Degrees
    /* GetMicroseconds() at the last sentence that carried a position. Compare
     * it only by unsigned subtraction from a later reading - at microsecond
     * resolution a uint32_t wraps every ~71.6 minutes, and (now - then) stays
     * correct across that wrap while an absolute comparison does not. Same
     * idiom as nav.c and control.c. */
    uint32_t usLastFix;
    uint8_t fixType; // 0 = no fix, 1 = dead reckoning, 2 = 2D fix, 3 = 3D fix
    uint8_t satellitesInUse;
} GpsData_t;


typedef struct GpsDriver_s {
    void* ctx;
    eSTATUS_t (*Read) (void* ctx, bool forcePolling, GpsData_t* pOutData);
    bool (*IsDataReady) (void* ctx);
    struct {
        /* Optional; zeroed means poll. Unlike the IMU, mag and baro there is no
         * pin and nothing to enable on a part: the receiver's UART interrupt is
         * already running, so a Notify here is raised from the RX ISR once an
         * assembled sentence terminates. */
        DataReadySignal_t signal;
    } cfg;
} GpsDriver_t;

/*
 * Fill pOutDriver->cfg first; this reads it and does NOT clear the struct, the
 * same contract UartPort_Init and SpiDev_Init keep.
 */
eSTATUS_t GpsDrv_Init (GpsDriver_t* pOutDriver);


#endif // DRIVERS_GPS_GPS_DRIVER_H