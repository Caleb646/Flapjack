#include "drivers/sim_link/sim_link.h"

#include "target.h"

#include "core/core.h"

#include "drivers/serial/serial_link.h"

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

static SemaphoreHandle_t s_magSem;      // given on each fresh SensorData (mag consumer)
static SemaphoreHandle_t s_baroSem;     // given on each fresh BaroData (its own frame, own rate)

static SensorSlot_t s_sensor;           // latest sample (critical-section guarded)
static volatile bool s_haveSensor;
static volatile uint32_t s_sensorCount;

static float s_baroPa;                  // latest BaroData (critical-section guarded)
static float s_baroTempC;

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
    s_sensorCount++;
    taskEXIT_CRITICAL ();
    (void)xSemaphoreGive (s_magSem);
}

static void SimLink_OnBaro (uint8_t const* pPayload, uint8_t len) {
    BaroData msg = BaroData_init_zero;
    pb_istream_t is = pb_istream_from_buffer (pPayload, len);
    if (!pb_decode (&is, BaroData_fields, &msg)) {
        return;
    }
    taskENTER_CRITICAL ();
    s_baroPa    = msg.pressure_pa;
    s_baroTempC = msg.temperature_c;
    taskEXIT_CRITICAL ();
    (void)xSemaphoreGive (s_baroSem);
}

eSTATUS_t SimLink_Init (void) {

    s_magSem  = xSemaphoreCreateBinary ();
    s_baroSem = xSemaphoreCreateBinary ();
    if (!s_magSem || !s_baroSem) {
        return eSTATUS_FAILURE;
    }

    /* FC->PC ids (3-5) are never expected inbound, so they get no handler. */
    if (STATUS_FAIL (SerialLink_RegisterHandler (SIM_MSG_SENSOR, SimLink_OnSensor))) {
        return eSTATUS_FAILURE;
    }
    if (STATUS_FAIL (SerialLink_RegisterHandler (SIM_MSG_BARO, SimLink_OnBaro))) {
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

/* --- sensor consumer API --------------------------------------------------- */

bool SimLink_WaitMag (float mag[3], uint32_t timeoutTicks) {
    if (xSemaphoreTake (s_magSem, timeoutTicks) != pdTRUE) {
        return false;
    }
    taskENTER_CRITICAL ();
    memcpy (mag, s_sensor.mag, sizeof (s_sensor.mag));
    taskEXIT_CRITICAL ();
    return true;
}

bool SimLink_WaitBaro (float* pPressurePa, float* pTemperatureC, uint32_t timeoutTicks) {
    if (xSemaphoreTake (s_baroSem, timeoutTicks) != pdTRUE) {
        return false;
    }
    taskENTER_CRITICAL ();
    *pPressurePa   = s_baroPa;
    *pTemperatureC = s_baroTempC;
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
