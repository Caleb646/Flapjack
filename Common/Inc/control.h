#ifndef CONTROL_H
#define CONTROL_H

#include "common.h"
#include "hal.h"
#include "log/logger.h"
#include "mem/mem.h"

#define COMMAND_TOTAL_SIZE  8U
#define COMMAND_HEADER_SIZE 1U
#define COMMAND_DATA_SIZE   (COMMAND_TOTAL_SIZE - COMMAND_HEADER_SIZE)

typedef uint8_t eCMD_FLIGHT_MODE_t;
enum { eCMD_FLIGHT_MODE_HOVER = 0, eCMD_FLIGHT_MODE_AIRPLANE };

typedef uint8_t eCMD_OP_STATE_t;
enum {
    eCMD_OP_STATE_STOPPED = 0,
    eCMD_OP_STATE_RUNNING,
    eCMD_OP_STATE_ERROR,

    eNUMBER_OF_OP_STATES
};

typedef uint8_t eCMD_t;
enum {
    eCMD_TYPE_EMPTY = 0, // represents invalid command
    eCMD_TYPE_CHANGE_OP_STATE,
    eCMD_TYPE_CHANGE_FLIGHT_MODE,
    eCMD_TYPE_CHANGE_VELOCITY,
    eCMD_TYPE_CHANGE_PID,

    eNUMBER_OF_CMD_TYPES
};

typedef int8_t cmd_velocity_t;
typedef uint16_t cmd_pid_t;
typedef uint8_t eCMD_PID_t;
enum {
    eCMD_PID_ROLL = 0,
    eCMD_PID_PITCH,
    eCMD_PID_YAW,
    eCMD_PID_THROTTLE
};

typedef struct {
    eCMD_t commandType;
} CommandHeader;

typedef struct {
    CommandHeader header;
    // uint8_t data[COMMAND_DATA_SIZE];
    uint8_t unused1;
    uint8_t unused2;
    uint8_t unused3;
    uint8_t unused4;
    uint8_t unused5;
    uint8_t unused6;
    uint8_t unused7;
} EmptyCommand;

typedef struct {
    CommandHeader header;
    eCMD_OP_STATE_t requestedState;
} ChangeOpStateCmd;

typedef struct {
    CommandHeader header;
    eCMD_FLIGHT_MODE_t flightMode;
} ChangeFlightModeCmd;

typedef struct {
    CommandHeader header;
    // uint8_t padding;
    cmd_velocity_t vThrottle; // -100 to 100
    cmd_velocity_t vRight;    // -100 to 100
    cmd_velocity_t vForward;  // -100 to 100
    // uint8_t padding[4];
} ChangeVelocityCmd;

typedef struct {
    CommandHeader header;
    eCMD_PID_t pidType;
    union {
        cmd_pid_t v;
        cmd_pid_t roll;
        cmd_pid_t pitch;
        cmd_pid_t yaw;
        cmd_pid_t throttle;
    } P; // 0 to 65,535

    union {
        cmd_pid_t v;
        cmd_pid_t roll;
        cmd_pid_t pitch;
        cmd_pid_t yaw;
        cmd_pid_t throttle;
    } I; // 0 to 65,535

    union {
        cmd_pid_t v;
        cmd_pid_t roll;
        cmd_pid_t pitch;
        cmd_pid_t yaw;
        cmd_pid_t throttle;
    } D; // 0 to 65,535
} ChangePIDCmd;

typedef struct {
    eCMD_FLIGHT_MODE_t flightMode;
    eCMD_OP_STATE_t opState;
} FCState;


STATIC_ASSERT (sizeof (EmptyCommand) <= COMMAND_TOTAL_SIZE, "");
STATIC_ASSERT (sizeof (ChangeOpStateCmd) <= sizeof (EmptyCommand), "");
STATIC_ASSERT (sizeof (ChangeFlightModeCmd) <= sizeof (EmptyCommand), "");
STATIC_ASSERT (sizeof (ChangeVelocityCmd) <= sizeof (EmptyCommand), "");
STATIC_ASSERT (sizeof (ChangePIDCmd) <= sizeof (EmptyCommand), "");
STATIC_ASSERT (sizeof (FCState) <= MEM_SHARED_FLIGHT_STATE_TOTAL_LEN, "");

typedef BOOL_t (*OpStateTransitionHandler_t) (FCState curState);
typedef eSTATUS_t (*CmdHandler_t) (EmptyCommand cmd);

eSTATUS_t ControlInit (void);
eSTATUS_t ControlStart (UART_HandleTypeDef* huart);
eSTATUS_t ControlProcess_RawCmds (void);
eSTATUS_t ControlProcess_Cmds (void);
eSTATUS_t ControlRegister_OPStateTransitionHandler (
eCMD_OP_STATE_t fromState,
eCMD_OP_STATE_t toState,
OpStateTransitionHandler_t handler);
eSTATUS_t ControlRegister_CmdHandler (eCMD_t cmdType, CmdHandler_t handler);
char const* ControlOpState2Char (eCMD_OP_STATE_t opState);
char const* ControlCmdType2Char (eCMD_t commandType);
FCState ControlGetCopyFCState (void);
eSTATUS_t ControlUpdateFCState (FCState const* pNewState);
eCMD_OP_STATE_t ControlGetOpState (void);

#endif /* CONTROL_H */