#include "tasks/shell/shell.h"

#include "target.h"

#include "core/core.h"

#include "drivers/serial/serial_link.h"

#include "umsg_tune.h"
#include "umsg_arming.h"

#include "flapjack.pb.h"
#include <pb_decode.h>

#include <stdbool.h>
#include <stdint.h>

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

    /*
     * Micro-units, not %f: tfp_format handles u/d/x/c/s only, so a %f prints
     * nothing at all (and does not consume its vararg). Scaling to an integer
     * is the convention the LOG_*_FLOATS macros already use. 1e6 because the
     * rate gains run to ~1e-6.
     */
    LOG_INFO (
    "Shell: set pid axis=%d gain=%d value=%d e-6",
    (int)pCmd->axis,
    (int)pCmd->gain,
    (int)(pCmd->value * 1000000.0F)
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

/*
 * Runs in the SerialLink RX task once a framed SERIAL_MSG_SHELL_CMD passes CRC,
 * so the payload is already a whole, verified Command.
 */
static void Shell_OnCommand (uint8_t const* pPayload, uint8_t len) {

    Command cmd     = Command_init_zero;
    pb_istream_t in = pb_istream_from_buffer (pPayload, len);
    if (!pb_decode (&in, Command_fields, &cmd)) {
        LOG_ERROR ("Shell: failed to decode command");
        return;
    }

    switch (cmd.which_cmd) {
    case Command_set_pid_tag: Shell_HandleSetPid (&cmd.cmd.set_pid); break;
    case Command_arm_disarm_tag: Shell_HandleArm (&cmd.cmd.arm_disarm); break;
    default: LOG_WARN ("Shell: unknown command tag %d", (int)cmd.which_cmd); break;
    }
}

eSTATUS_t Shell_Init (void) {

    return SerialLink_RegisterHandler (SERIAL_MSG_SHELL_CMD, Shell_OnCommand);
}
