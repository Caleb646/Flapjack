#ifndef DRIVERS_SIM_LINK_SIM_LINK_H
#define DRIVERS_SIM_LINK_SIM_LINK_H

/*
 * Protobuf codec for the JSBSim HIL link (CM7 only). Encodes and decodes the
 * messages defined in msgs/proto/defs/sim.proto; the wire itself belongs to
 * drivers/serial/serial_link.c, which owns the UART, the framing and the
 * queueing. All sim driver backends (imu/mag/servo/motor/rx) talk to the PC
 * only through this module:
 *
 *   PC -> FC : SensorData, RcInput   (handlers registered with SerialLink)
 *   FC -> PC : ServoCmd, MotorCmd, Telemetry (sent by TX helpers)
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
#define SIM_MSG_RC        2U
#define SIM_MSG_SERVO     3U
#define SIM_MSG_MOTOR     4U
#define SIM_MSG_TELEMETRY 5U

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

// --- Actuator / telemetry TX (FC -> PC) -------------------------------------
eSTATUS_t SimLink_SendServos (float const* anglesRad, uint32_t count);
eSTATUS_t SimLink_SendThrottles (float const* throttles, uint32_t count);
eSTATUS_t SimLink_SendTelemetry (float const eulerDeg[3], bool armed, uint32_t imuCount);

#endif // DRIVERS_SIM_LINK_SIM_LINK_H
