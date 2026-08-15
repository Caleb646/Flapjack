#include "drivers/sim_link/sim_link.h"

#include "target.h"

#include "core/core.h"

#include "drivers/serial/uart.h"
#include "drivers/rx/rx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stream_buffer.h"

#include "sim.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

#include <string.h>

#define SIM_MAGIC0      0xAAU
#define SIM_MAGIC1      0x55U
#define SIM_MAX_PAYLOAD 96U   /* RcInput_size, the largest message */
#define SIM_MAX_FRAME   (4U + SIM_MAX_PAYLOAD + 1U)
#define SIM_RX_STREAM   512U

typedef struct {
    float accel[3];
    float gyro[3];
    float mag[3];
} SensorSlot_t;

static UartPort_t s_port;
static StreamBufferHandle_t s_rxStream;
static SemaphoreHandle_t s_sensorSem;   // given on each fresh SensorData (IMU consumer)
static SemaphoreHandle_t s_magSem;      // ditto for the mag consumer - separate because a
                                        // binary semaphore only ever wakes one waiter
static SemaphoreHandle_t s_txMutex;     // serialises outgoing frames

static SensorSlot_t s_sensor;           // latest sample (critical-section guarded)
static volatile bool s_haveSensor;
static volatile uint32_t s_sensorCount;

/* CRC8 (poly 0x07, init 0x00) over (msg_id, len, payload). Mirrors the PC. */
static uint8_t SimCrc8 (uint8_t const* pData, uint32_t len) {
    uint8_t crc = 0U;
    for (uint32_t i = 0; i < len; ++i) {
        crc ^= pData[i];
        for (uint8_t b = 0; b < 8U; ++b) {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static void SimLink_RxIsr (uint8_t const* pData, uint32_t len) {
    BaseType_t woken = pdFALSE;
    (void)xStreamBufferSendFromISR (s_rxStream, pData, len, &woken);
    portYIELD_FROM_ISR (woken);
}

eSTATUS_t SimLink_Init (void) {

    s_rxStream  = xStreamBufferCreate (SIM_RX_STREAM, 1U);
    s_sensorSem = xSemaphoreCreateBinary ();
    s_magSem    = xSemaphoreCreateBinary ();
    s_txMutex   = xSemaphoreCreateMutex ();
    if (!s_rxStream || !s_sensorSem || !s_magSem || !s_txMutex) {
        return eSTATUS_FAILURE;
    }

    s_port.cfg.id          = BRD_GET_UART_ID (SERIAL_DEBUG);
    s_port.cfg.baudRate    = SIM_LINK_BAUD;
    s_port.cfg.rxCallback  = SimLink_RxIsr;
    s_port.cfg.irqPriority = 6U;
    return UartPort_Init (&s_port);
}

/* --- frame dispatch -------------------------------------------------------- */

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

static void SimLink_Dispatch (uint8_t id, uint8_t const* pPayload, uint8_t len) {
    switch (id) {
    case SIM_MSG_SENSOR: SimLink_OnSensor (pPayload, len); break;
    case SIM_MSG_RC: SimLink_OnRc (pPayload, len); break;
    default: break;   /* FC->PC ids are not expected inbound */
    }
}

/* --- RX parser task -------------------------------------------------------- */

typedef enum { ST_MAGIC0, ST_MAGIC1, ST_ID, ST_LEN, ST_PAYLOAD, ST_CRC } RxState_t;

void SimLink_RxTask (void* args) {
    (void)args;
    RxState_t state = ST_MAGIC0;
    uint8_t id = 0, len = 0, idx = 0;
    uint8_t payload[SIM_MAX_PAYLOAD];

    for (;;) {
        uint8_t byte;
        if (xStreamBufferReceive (s_rxStream, &byte, 1U, portMAX_DELAY) != 1U) {
            continue;
        }
        switch (state) {
        case ST_MAGIC0:
            state = (byte == SIM_MAGIC0) ? ST_MAGIC1 : ST_MAGIC0;
            break;
        case ST_MAGIC1:
            state = (byte == SIM_MAGIC1) ? ST_ID : ((byte == SIM_MAGIC0) ? ST_MAGIC1 : ST_MAGIC0);
            break;
        case ST_ID:
            id    = byte;
            state = ST_LEN;
            break;
        case ST_LEN:
            len = byte;
            idx = 0;
            if (len > SIM_MAX_PAYLOAD) {
                state = ST_MAGIC0;   /* bogus length, resync */
            } else {
                state = (len == 0) ? ST_CRC : ST_PAYLOAD;
            }
            break;
        case ST_PAYLOAD:
            payload[idx++] = byte;
            if (idx >= len) {
                state = ST_CRC;
            }
            break;
        case ST_CRC: {
            /* CRC is over the concatenation (id, len, payload). */
            uint8_t buf[2 + SIM_MAX_PAYLOAD];
            buf[0] = id;
            buf[1] = len;
            memcpy (&buf[2], payload, len);
            if (SimCrc8 (buf, (uint32_t)len + 2U) == byte) {
                SimLink_Dispatch (id, payload, len);
            }
            state = ST_MAGIC0;
            break;
        }
        default: state = ST_MAGIC0; break;
        }
    }
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
    uint8_t len = (uint8_t)os.bytes_written;

    uint8_t frame[SIM_MAX_FRAME];
    frame[0] = SIM_MAGIC0;
    frame[1] = SIM_MAGIC1;
    frame[2] = id;
    frame[3] = len;
    memcpy (&frame[4], payload, len);
    frame[4 + len] = SimCrc8 (&frame[2], (uint32_t)len + 2U);   /* over id,len,payload */

    eSTATUS_t status = eSTATUS_FAILURE;
    if (xSemaphoreTake (s_txMutex, portMAX_DELAY) == pdTRUE) {
        status = UartPort_Write (&s_port, frame, (uint32_t)len + 5U);
        (void)xSemaphoreGive (s_txMutex);
    }
    return status;
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
