#ifndef SENSORS_GPS_H
#define SENSORS_GPS_H

#include "core/core.h"

#include "drivers/gps/gpsdrv.h"

#include <stdbool.h>

/*
 * How long a fix stays trustworthy without a fresh positional sentence.
 *
 * A receiver that loses lock says so (void RMC / GGA fix_quality 0) and the
 * driver reports that immediately. One that is unplugged, browns out or loses
 * its antenna says nothing at all - so silence has to be timed out, or the last
 * good fix stays the newest value in every subscriber's cache forever.
 *
 * 2 s is ~20 missed fixes from a 10 Hz receiver: long enough not to trip on
 * ordinary jitter, short enough that a dead antenna is noticed promptly. The RC
 * path uses 1 s (RX_LINK_TIMEOUT_US), fixed by the CRSF spec; nothing fixes this
 * one, so it is a judgement call.
 */
#define GPS_FIX_TIMEOUT_US 2000000U

typedef struct {
    GpsDriver_t driver;
    GpsData_t data;
} Gps_t;

eSTATUS_t Gps_Init(Gps_t* pOutGps);
eSTATUS_t Gps_Update(Gps_t* pGps);

/*
 * True only if the receiver reports a fix AND that fix is recent. Covers both
 * ways a fix can stop being real - an explicit loss of lock, and a receiver that
 * simply stops talking. The counterpart to Rx_IsLinkUp() on the RC path.
 */
bool Gps_HasFix(Gps_t const* pGps);

#endif // SENSORS_GPS_H
