#ifndef DRIVERS_SIM_LINK_SIM_LINK_H
#define DRIVERS_SIM_LINK_SIM_LINK_H

/*
 * Protobuf codec for the JSBSim HIL link (CM7 only). Encodes the messages
 * defined in msgs/proto/defs/sim.proto; the wire itself belongs to
 * drivers/serial/serial_link.c, which owns the UART, the framing and the
 * queueing. The sim actuator backends (servo/motor) talk to the PC only through
 * this module:
 *
 *   FC -> PC : ServoCmd, MotorCmd, Telemetry (sent by TX helpers)
 *
 * NOTHING arrives here any more. Every inbound path was retired in favour of a
 * wire the firmware has to parse for real:
 *
 *   - RC used to be an RcInput frame decoded straight into g_Rx.channels, which
 *     skipped the UART ISR, the CRSF deframing, the CRC and the channel
 *     mapping. It is real CRSF on the RX UART now.
 *   - IMU, mag and baro used to be SensorData and BaroData, decoded past the
 *     drivers into sim backends. All three parts are emulated in Renode now
 *     (Scripts/renode/*.cs) and read through their real drivers over emulated
 *     SPI, so the register maps, the chip selects and the barometer's
 *     compensation polynomial are all exercised rather than stepped over.
 *   - GPS never had a frame here, for the same reason.
 *
 * The module always compiles; SIM_HIL only gates its activation in main.c - so
 * hardware builds keep it type-checked without ever running it.
 */

#include "core/core.h"

#include <stdbool.h>
#include <stdint.h>

// Frame message ids (must match the PC bridge and sim.proto comment). These
// share one namespace with every other SerialLink client - see
// SERIAL_MSG_SHELL_CMD in drivers/serial/serial_link.h.
/* 1 is retired (was SensorData - accel/gyro/mag). 2 is retired (was RcInput).
 * 7 is retired (was BaroData). Do not reuse any of them: an older bridge on the
 * other end of the wire would still be sending them, and they would decode as
 * whatever took their place. */
#define SIM_MSG_SERVO     3U
#define SIM_MSG_MOTOR     4U
#define SIM_MSG_TELEMETRY 5U
/* 6 is SERIAL_MSG_SHELL_CMD (drivers/serial/serial_link.h) - one namespace. */
/* 7 was the LAST usable id: SerialLink_RegisterHandler rejects msgId >=
 * SERIAL_LINK_MAX_MSG_ID (8). Retiring 1, 2 and 7 does not free them, so a new
 * frame still has to raise the ceiling and grow the handler table. */

/* GPS has no id here on purpose - it arrives as real NMEA on the GPS UART, the
 * same reasoning that moved RC onto real CRSF. See tasks/gps/. */

// One-time init. Call once from main.c before the scheduler starts.
eSTATUS_t SimLink_Init (void);

// --- Actuator / telemetry TX (FC -> PC) -------------------------------------
eSTATUS_t SimLink_SendServos (float const* anglesRad, uint32_t count);
eSTATUS_t SimLink_SendThrottles (float const* throttles, uint32_t count);

/*
 * Telemetry payload, gathered by tasks/sim/sim_telemetry.c.
 *
 * The baro/gps members are a loopback of what the FC decoded, read from the
 * umsg topics rather than from this module's own decode slots - deliberately,
 * so the round trip the bridge asserts on covers the driver, the device and the
 * publish, not just the frame. A struct rather than 11 positional args.
 */
typedef struct {
    /* The vectors are arrays, not pointers, so their length is part of the type
     * rather than a comment nobody can check - and so the struct OWNS its data
     * instead of borrowing three buffers that have to outlive the call. */
    float    eulerDeg[3];   // deg [roll, pitch, yaw]
    bool     armed;
    uint32_t imuCount;      // sensors_imu_status, i.e. samples the IMU task read
    bool     rcLinkUp;

    float    baroPa;
    uint32_t baroCount;

    double   gpsLat;
    double   gpsLon;
    float    gpsAlt;
    uint32_t gpsSats;
    uint32_t gpsCount;

    /* Estimator output, NOT a loopback - see the nav_* comment in sim.proto.
     * NED, down positive, straight from umsg_nav_state_t. */
    float    posNed[3];   // metres
    float    velNed[3];   // metres/second
    uint32_t navValid;    // NAV_VALID_* bitmask
    float    accelResidualDeg;  // estimator innovation, deg (Nav_AccelResidualDeg)
} SimLinkTelemetry_t;

eSTATUS_t SimLink_SendTelemetry (SimLinkTelemetry_t const* pTelemetry);

#endif // DRIVERS_SIM_LINK_SIM_LINK_H
