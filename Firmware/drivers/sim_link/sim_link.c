#include "drivers/sim_link/sim_link.h"

#include "target.h"

#include "core/core.h"

#include "drivers/serial/serial_link.h"

#include "sim.pb.h"
#include "pb_encode.h"

#include <string.h>

#define SIM_MAX_PAYLOAD SERIAL_LINK_MAX_PAYLOAD

eSTATUS_t SimLink_Init (void) {

    /* Nothing to register: every PC->FC frame is retired, and the FC->PC ids
     * (3-5) are never expected inbound. Kept as the module's entry point
     * because main.c calls it under SIM_HIL and a frame may come back. */
    return eSTATUS_SUCCESS;
}

/* --- TX helpers ------------------------------------------------------------ */

static eSTATUS_t SimLink_SendFrame (uint8_t id, pb_msgdesc_t const* fields, void const* msg) {
    uint8_t payload[SIM_MAX_PAYLOAD];
    pb_ostream_t os = pb_ostream_from_buffer (payload, sizeof (payload));
    if (!pb_encode (&os, fields, msg)) {
        return eSTATUS_FAILURE;
    }
    /* SerialLink owns the framing, the queueing and the UART. */
    return SerialLink_SendFrame (id, payload, (uint8_t)os.bytes_written);
}

eSTATUS_t SimLink_SendServos (float const* anglesRad, uint32_t count) {
    ServoCmd msg = ServoCmd_init_zero;
    if (count > 8U) {
        count = 8U;
    }
    msg.angle_count = (pb_size_t)count;
    memcpy (msg.angle, anglesRad, count * sizeof (float));
    return SimLink_SendFrame (SIM_MSG_SERVO, ServoCmd_fields, &msg);
}

eSTATUS_t SimLink_SendThrottles (float const* throttles, uint32_t count) {
    MotorCmd msg = MotorCmd_init_zero;
    if (count > 8U) {
        count = 8U;
    }
    msg.throttle_count = (pb_size_t)count;
    memcpy (msg.throttle, throttles, count * sizeof (float));
    return SimLink_SendFrame (SIM_MSG_MOTOR, MotorCmd_fields, &msg);
}

eSTATUS_t SimLink_SendTelemetry (SimLinkTelemetry_t const* pTelemetry) {
    if (!pTelemetry) {
        return eSTATUS_NULL_ARG;
    }
    Telemetry msg = Telemetry_init_zero;
    memcpy (msg.euler, pTelemetry->eulerDeg, sizeof (msg.euler));
    msg.armed       = pTelemetry->armed;
    msg.imu_count   = pTelemetry->imuCount;
    msg.rc_link_up  = pTelemetry->rcLinkUp;
    msg.baro_pa     = pTelemetry->baroPa;
    msg.baro_count  = pTelemetry->baroCount;
    msg.gps_lat     = pTelemetry->gpsLat;
    msg.gps_lon     = pTelemetry->gpsLon;
    msg.gps_alt     = pTelemetry->gpsAlt;
    msg.gps_sats    = pTelemetry->gpsSats;
    msg.gps_count   = pTelemetry->gpsCount;
    memcpy (msg.nav_pos_ned, pTelemetry->posNed, sizeof (msg.nav_pos_ned));
    memcpy (msg.nav_vel_ned, pTelemetry->velNed, sizeof (msg.nav_vel_ned));
    msg.nav_valid   = pTelemetry->navValid;
    return SimLink_SendFrame (SIM_MSG_TELEMETRY, Telemetry_fields, &msg);
}
