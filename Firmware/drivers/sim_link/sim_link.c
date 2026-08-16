#include "drivers/sim_link/sim_link.h"

#include "target.h"

#include "core/core.h"

#include "drivers/serial/serial_link.h"
#include "drivers/rx/rx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "sim.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

#include <string.h>

#define SIM_MAX_PAYLOAD SERIAL_LINK_MAX_PAYLOAD

typedef struct {
    float accel[3];
    float gyro[3];
    float mag[3];
} SensorSlot_t;

static SemaphoreHandle_t s_sensorSem;   // given on each fresh SensorData (IMU consumer)
static SemaphoreHandle_t s_magSem;      // ditto for the mag consumer - separate because a
                                        // binary semaphore only ever wakes one waiter

static SensorSlot_t s_sensor;           // latest sample (critical-section guarded)
static volatile bool s_haveSensor;
static volatile uint32_t s_sensorCount;

/* --- frame handlers (called from the SerialLink RX task) ------------------- */

static void SimLink_OnSensor (uint8_t const* pPayload, uint8_t len) {
    SensorData msg = SensorData_init_zero;
    pb_istream_t is = pb_istream_from_buffer (pPayload, len);
    if (!pb_decode (&is, SensorData_fields, &msg)) {
        return;
    }
    taskENTER_CRITICAL ();
    memcpy (s_sensor.accel, msg.accel, sizeof (s_sensor.accel));
    memcpy (s_sensor.gyro, msg.gyro, sizeof (s_sensor.gyro));
    memcpy (s_sensor.mag, msg.mag, sizeof (s_sensor.mag));
    s_haveSensor = true;
    taskEXIT_CRITICAL ();
    (void)xSemaphoreGive (s_sensorSem);
    (void)xSemaphoreGive (s_magSem);
}

static void SimLink_OnRc (uint8_t const* pPayload, uint8_t len) {
    RcInput msg = RcInput_init_zero;
    pb_istream_t is = pb_istream_from_buffer (pPayload, len);
    if (!pb_decode (&is, RcInput_fields, &msg)) {
        return;
    }
    uint32_t n = msg.channels_count;
    if (n > RC_MAX_CHANNELS) {
        n = RC_MAX_CHANNELS;
    }
    taskENTER_CRITICAL ();
    for (uint32_t i = 0; i < n; ++i) {
        g_Rx.channels[i] = msg.channels[i];
    }
    taskEXIT_CRITICAL ();
}

eSTATUS_t SimLink_Init (void) {

    s_sensorSem = xSemaphoreCreateBinary ();
    s_magSem    = xSemaphoreCreateBinary ();
    if (!s_sensorSem || !s_magSem) {
        return eSTATUS_FAILURE;
    }

    /* FC->PC ids (3-5) are never expected inbound, so they get no handler. */
    if (STATUS_FAIL (SerialLink_RegisterHandler (SIM_MSG_SENSOR, SimLink_OnSensor)) ||
        STATUS_FAIL (SerialLink_RegisterHandler (SIM_MSG_RC, SimLink_OnRc))) {
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

/* --- sensor consumer API --------------------------------------------------- */

bool SimLink_WaitImu (float accel[3], float gyro[3], float mag[3], uint32_t timeoutTicks) {
    if (xSemaphoreTake (s_sensorSem, timeoutTicks) != pdTRUE) {
        return false;
    }
    taskENTER_CRITICAL ();
    memcpy (accel, s_sensor.accel, sizeof (s_sensor.accel));
    memcpy (gyro, s_sensor.gyro, sizeof (s_sensor.gyro));
    memcpy (mag, s_sensor.mag, sizeof (s_sensor.mag));
    s_sensorCount++;   /* count samples actually consumed by the IMU driver */
    taskEXIT_CRITICAL ();
    return true;
}

bool SimLink_WaitMag (float mag[3], uint32_t timeoutTicks) {
    if (xSemaphoreTake (s_magSem, timeoutTicks) != pdTRUE) {
        return false;
    }
    taskENTER_CRITICAL ();
    memcpy (mag, s_sensor.mag, sizeof (s_sensor.mag));
    taskEXIT_CRITICAL ();
    return true;
}

uint32_t SimLink_GetSensorCount (void) {
    return s_sensorCount;
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

eSTATUS_t SimLink_SendTelemetry (float const eulerDeg[3], bool armed, uint32_t imuCount) {
    Telemetry msg = Telemetry_init_zero;
    memcpy (msg.euler, eulerDeg, sizeof (msg.euler));
    msg.armed     = armed;
    msg.imu_count = imuCount;
    return SimLink_SendFrame (SIM_MSG_TELEMETRY, Telemetry_fields, &msg);
}
