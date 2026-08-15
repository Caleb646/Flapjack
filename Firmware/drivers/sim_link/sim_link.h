#ifndef DRIVERS_SIM_LINK_SIM_LINK_H
#define DRIVERS_SIM_LINK_SIM_LINK_H

/*
 * Shared transport for the JSBSim HIL link (CM7 only).
 *
 * Owns the debug UART and is the single point that frames/deframes the
 * protobuf messages defined in msgs/proto/defs/sim.proto. All sim driver
 * backends (imu/mag/servo/motor/rx) talk to the PC only through this module:
 *
 *   PC -> FC : SensorData, RcInput   (parsed by the RX task, pushed to slots)
 *   FC -> PC : ServoCmd, MotorCmd, Telemetry (sent by TX helpers)
 *
 * Wire frame:  [0xAA][0x55][msg_id][len][payload...][crc8]
 *   crc8 is computed over (msg_id, len, payload). len is the payload byte
 *   count (<= 96). Both ends are little-endian; payload is nanopb-encoded.
 *
 * The module always compiles; SIM_HIL only gates its activation in main.c -
 * claiming the debug UART and starting the RX task - so hardware builds keep it
 * type-checked without ever running it.
 */

#include "core/core.h"

#include <stdbool.h>
#include <stdint.h>

// Frame message ids (must match the PC bridge and sim.proto comment).
#define SIM_MSG_SENSOR    1U
#define SIM_MSG_RC        2U
#define SIM_MSG_SERVO     3U
#define SIM_MSG_MOTOR     4U
#define SIM_MSG_TELEMETRY 5U

// UART baud for the sim link. Raised above the 230400 debug default so the
// sensor stream + framing fits comfortably at a few hundred Hz (460800 carries
// the ~34 kB/s PC->FC stream at --rate 400 with headroom).
#ifndef SIM_LINK_BAUD
#define SIM_LINK_BAUD 460800U
#endif

// One-time init: claims the debug UART and creates the RX/TX primitives.
// Call once from main.c before the scheduler starts.
eSTATUS_t SimLink_Init (void);

// RX parser task body: drains the UART stream, deframes, decodes, dispatches.
void SimLink_RxTask (void* args);

// --- Sensor RX (PC -> FC) ---------------------------------------------------
// Block up to timeoutTicks for the next SensorData; copies the latest sample.
// Returns true if a fresh sample was signalled, false on timeout.
bool SimLink_WaitSensor (float accel[3], float gyro[3], float mag[3], uint32_t timeoutTicks);
// Non-blocking copy of the latest magnetic field. Returns true once any
// SensorData has been received.
bool SimLink_GetMag (float mag[3]);
// Count of SensorData samples consumed by the IMU driver (pacing telemetry).
uint32_t SimLink_GetSensorCount (void);

// --- Actuator / telemetry TX (FC -> PC) -------------------------------------
eSTATUS_t SimLink_SendServos (float const* anglesRad, uint32_t count);
eSTATUS_t SimLink_SendThrottles (float const* throttles, uint32_t count);
eSTATUS_t SimLink_SendTelemetry (float const eulerDeg[3], bool armed, uint32_t imuCount);

#endif // DRIVERS_SIM_LINK_SIM_LINK_H
