#ifndef DRIVERS_SIM_LINK_SIM_LINK_H
#define DRIVERS_SIM_LINK_SIM_LINK_H

/*
 * Protobuf codec for the JSBSim HIL link (CM7 only). Encodes and decodes the
 * messages defined in msgs/proto/defs/sim.proto; the wire itself belongs to
 * drivers/serial/serial_link.c, which owns the UART, the framing and the
 * queueing. All sim driver backends (imu/mag/servo/motor/rx) talk to the PC
 * only through this module:
 *
 *   PC -> FC : SensorData             (handler registered with SerialLink)
 *   FC -> PC : ServoCmd, MotorCmd, Telemetry (sent by TX helpers)
 *
 * RC is deliberately NOT here. It used to arrive as an RcInput frame that was
 * decoded straight into g_Rx.channels, which skipped the receiver driver
 * entirely - the UART ISR, CRSF deframing, the CRC and the channel mapping were
 * all untested by the SIL. The bridge now sends real CRSF frames to the RX UART
 * instead, so the SIL exercises the same path the aircraft flies.
 *
 * The module always compiles; SIM_HIL only gates its activation in main.c -
 * registering the inbound handlers - so hardware builds keep it type-checked
 * without ever running it.
 */

#include "core/core.h"

#include <stdbool.h>
#include <stdint.h>

// Frame message ids (must match the PC bridge and sim.proto comment). These
// share one namespace with every other SerialLink client - see
// SERIAL_MSG_SHELL_CMD in drivers/serial/serial_link.h.
#define SIM_MSG_SENSOR    1U
/* 2 is retired (was RcInput; RC now arrives as CRSF on the RX UART). Do not
 * reuse it - an older bridge on the other end of the wire would still be
 * sending it, and it would decode as whatever took its place. */
#define SIM_MSG_SERVO     3U
#define SIM_MSG_MOTOR     4U
#define SIM_MSG_TELEMETRY 5U
/* 6 is SERIAL_MSG_SHELL_CMD (drivers/serial/serial_link.h) - one namespace. */
#define SIM_MSG_BARO      7U
/* 7 is the LAST usable id: SerialLink_RegisterHandler rejects msgId >=
 * SERIAL_LINK_MAX_MSG_ID (8). Only 0 is left; the next frame after that has to
 * raise the ceiling and grow the handler table. */

/* GPS has no id here on purpose - it arrives as real NMEA on the GPS UART, the
 * same reasoning that moved RC onto real CRSF. See tasks/gps/. */

// One-time init: creates the sensor semaphores and registers the inbound frame
// handlers. Call once from main.c before the scheduler starts.
eSTATUS_t SimLink_Init (void);

// --- Sensor RX (PC -> FC) ---------------------------------------------------
// Each SensorData signals both waiters below. They take separate semaphores on
// purpose: a binary semaphore wakes exactly one task, so a shared one would let
// the IMU and mag consumers steal samples from each other.
//
// Block up to timeoutTicks for the next SensorData; copies the latest sample.
// Returns true if a fresh sample was signalled, false on timeout.
bool SimLink_WaitImu (float accel[3], float gyro[3], float mag[3], uint32_t timeoutTicks);
// As above, for the magnetic field only.
bool SimLink_WaitMag (float mag[3], uint32_t timeoutTicks);
// Count of SensorData samples consumed by the IMU driver (pacing telemetry).
uint32_t SimLink_GetSensorCount (void);

/*
 * Block up to timeoutTicks for the next BaroData; copies the latest sample.
 * Its own frame and its own semaphore, so Baro_Task is paced by the bridge's
 * baro rate (~50 Hz) rather than the 400 Hz SensorData stream.
 */
bool SimLink_WaitBaro (float* pPressurePa, float* pTemperatureC, uint32_t timeoutTicks);

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
    uint32_t imuCount;
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
} SimLinkTelemetry_t;

eSTATUS_t SimLink_SendTelemetry (SimLinkTelemetry_t const* pTelemetry);

#endif // DRIVERS_SIM_LINK_SIM_LINK_H
