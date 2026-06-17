#include "tasks/shell/shell.h"

#include "target.h"

#include "core/core.h"

#include "drivers/serial/uart.h"

#include "umsg_tune.h"
#include "umsg_arming.h"

#include "flapjack.pb.h"
#include <pb_decode.h>

#include <stdbool.h>
#include <stdint.h>

// ---------------------------------------------------------------------------
// RX state machine
// ---------------------------------------------------------------------------

typedef enum {
    STATE_WAIT_LEN,
    STATE_READING,
} RxState_t;

static RxState_t s_State = STATE_WAIT_LEN;
static uint8_t s_Len     = 0U;
static uint8_t s_Buf[128];
static uint8_t s_Read         = 0U;
static volatile bool s_HasMsg = false;

static void Shell_RxCb (uint8_t const* pData, uint32_t len) {

    for (uint32_t i = 0U; i < len; i++) {
        uint8_t b = pData[i];
        if (s_State == STATE_WAIT_LEN) {
            s_Len  = b;
            s_Read = 0U;
            if (s_Len > 0U && s_Len <= sizeof (s_Buf)) {
                s_State = STATE_READING;
            }
        } else {
            s_Buf[s_Read++] = b;
            if (s_Read >= s_Len) {
                s_HasMsg = true;
                s_State  = STATE_WAIT_LEN;
            }
        }
    }
}

static UartPort_t s_Port = {
    .cfg = {
        .id          = BRD_GET_UART_ID (SERIAL_DEBUG),
        .baudRate    = 0U, // preserve baud rate already set by SerialDebug_Init
        .rxCallback  = Shell_RxCb,
        .irqPriority = 6U,
    },
};

// ---------------------------------------------------------------------------
// Command handlers
// ---------------------------------------------------------------------------

static void Shell_HandleSetPid (SetPidCmd const* pCmd) {

    uint32_t axisIdx = 0U;
    switch (pCmd->axis) {
    case Axis_ROLL: axisIdx = AXIS_IDX_ROLL; break;
    case Axis_PITCH: axisIdx = AXIS_IDX_PITCH; break;
    case Axis_YAW: axisIdx = AXIS_IDX_YAW; break;
    case Axis_THROTTLE: axisIdx = AXIS_IDX_THROTTLE; break;
    default: LOG_WARN ("Shell: unknown axis %d", (int)pCmd->axis); return;
    }

    umsg_tune_gain_t gain;
    switch (pCmd->gain) {
    case PidGainType_KP: gain = TUNE_PID_GAIN_KP; break;
    case PidGainType_KI: gain = TUNE_PID_GAIN_KI; break;
    case PidGainType_KD: gain = TUNE_PID_GAIN_KD; break;
    case PidGainType_INTEGRAL_LIMIT: gain = TUNE_PID_GAIN_INTEGRAL_LIMIT; break;
    default: LOG_WARN ("Shell: unknown gain type %d", (int)pCmd->gain); return;
    }

    umsg_tune_pid_t msg = { .axis = (uint8_t)axisIdx, .gain = gain, .value = pCmd->value };
    umsg_tune_pid_publish (&msg);

    LOG_INFO (
    "Shell: set pid axis=%d gain=%d value=%f",
    (int)pCmd->axis,
    (int)pCmd->gain,
    (double)pCmd->value
    );
}

static void Shell_HandleArm (ArmCmd const* pCmd) {

    umsg_arming_request_t msg = { .arm = pCmd->arm ? 1U : 0U };
    umsg_arming_request_publish (&msg);
    LOG_INFO ("Shell: arm request %d", (int)pCmd->arm);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

eSTATUS_t Shell_Init (void) {

    return UartPort_Init (&s_Port);
}

eSTATUS_t Shell_Update (uint32_t usCurrentTime, uint32_t usDeltaTime) {

    (void)usCurrentTime;
    (void)usDeltaTime;

    if (!s_HasMsg) {
        return eSTATUS_SUCCESS;
    }
    s_HasMsg = false;

    Command cmd     = Command_init_zero;
    pb_istream_t in = pb_istream_from_buffer (s_Buf, s_Len);
    if (!pb_decode (&in, Command_fields, &cmd)) {
        LOG_ERROR ("Shell: failed to decode command");
        return eSTATUS_SUCCESS;
    }

    switch (cmd.which_cmd) {
    case Command_set_pid_tag: Shell_HandleSetPid (&cmd.cmd.set_pid); break;
    case Command_arm_disarm_tag: Shell_HandleArm (&cmd.cmd.arm_disarm); break;
    default: LOG_WARN ("Shell: unknown command tag %d", (int)cmd.which_cmd); break;
    }

    return eSTATUS_SUCCESS;
}
