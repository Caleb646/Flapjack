#ifndef CONTROL_H
#define CONTROL_H

#include "common.h"
#include "fcstate.h"
#include "hal.h"
#include "log/logger.h"
#include "mem/mem.h"
#include "peripheral/bus/uart.h"


typedef uint32_t eCMD_t;
enum {
    eCMD_NULL               = 0,
    eCMD_CHANGE_OP_STATE    = (1U << 8U),
    eCMD_CHANGE_FLIGHT_MODE = (1U << 9U),
    eCMD_CHANGE_VELOCITY    = (1U << 10U),
    eCMD_CHANGE_PID         = (1U << 11U),
};

typedef struct {
    eCMD_t commandType;
} CommandHeader_t;

typedef union {
    uint32_t raw;
    struct {
        uint16_t cState;
        uint16_t nState;
    } opstate;
} SubCommand_t;

#define CMD_TOTAL_SIZE_BYTES  16U
#define CMD_HEADER_SIZE_BYTES (sizeof (CommandHeader_t))
#define CMD_DATA_SIZE_BYTES   (CMD_TOTAL_SIZE_BYTES - CMD_HEADER_SIZE_BYTES)
#define eNUMBER_OF_CMD_TYPES  5U

typedef int8_t cmd_velocity_t;
typedef uint16_t cmd_pid_t;
typedef uint8_t eCMD_PID_t;
enum { eCMD_PID_ROLL = 0, eCMD_PID_PITCH, eCMD_PID_YAW, eCMD_PID_THROTTLE };


typedef struct {
    CommandHeader_t header;
    // uint8_t data[CMD_DATA_SIZE_BYTES];
    uint8_t unused1;
    uint8_t unused2;
    uint8_t unused3;
    uint8_t unused4;
    uint8_t unused5;
    uint8_t unused6;
    uint8_t unused7;
} DefaultCommand;

typedef struct {
    CommandHeader_t header;
    eOP_STATE_t requestedState;
} ChangeOpStateCmd;

typedef struct {
    CommandHeader_t header;
    eFLIGHT_MODE_t flightMode;
} ChangeFlightModeCmd;

typedef struct {
    CommandHeader_t header;
    // uint8_t padding;
    cmd_velocity_t vThrottle; // -100 to 100
    cmd_velocity_t vRight;    // -100 to 100
    cmd_velocity_t vForward;  // -100 to 100
    // uint8_t padding[4];
} ChangeVelocityCmd;

typedef struct {
    CommandHeader_t header;
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

STATIC_ASSERT (sizeof (DefaultCommand) <= CMD_TOTAL_SIZE_BYTES, "");
STATIC_ASSERT (sizeof (ChangeOpStateCmd) <= sizeof (DefaultCommand), "");
STATIC_ASSERT (sizeof (ChangeFlightModeCmd) <= sizeof (DefaultCommand), "");
STATIC_ASSERT (sizeof (ChangeVelocityCmd) <= sizeof (DefaultCommand), "");
STATIC_ASSERT (sizeof (ChangePIDCmd) <= sizeof (DefaultCommand), "");

// typedef bool (*OpStateTransitionHandler_t) (vFCState_t curState);
typedef eSTATUS_t (*CmdHandlerFn_t) (DefaultCommand cmd);

eSTATUS_t ControlInit (void);
eSTATUS_t ControlStart (void);
eSTATUS_t ControlProcess_RawCmds (void);
eSTATUS_t ControlProcess_Cmds (void);
bool ControlRegisterHandler (eCMD_t cmdType, SubCommand_t subCmd, CmdHandlerFn_t handler);
char const* ControlCmdType2Char (eCMD_t commandType);

#define CONTROL_INIT(pSTATUS)        \
    do {                             \
        *(pSTATUS) = ControlInit (); \
    } while (0)

#define CONTROL_REG_HANDLER(CMD_TYPE, HANDLER) \
    ControlRegisterHandler (CMD_TYPE, (SubCommand_t){ .raw = 0 }, HANDLER)

#define CONTROL_REG_OPSTATE_HANDLER(CUR_STATE, NEW_STATE, HANDLER) \
    ControlRegisterHandler (eCMD_CHANGE_OP_STATE, (SubCommand_t){ .opstate = { CUR_STATE, NEW_STATE } }, HANDLER)

#endif /* CONTROL_H */