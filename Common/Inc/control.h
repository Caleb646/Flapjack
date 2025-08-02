#ifndef CONTROL_H
#define CONTROL_H

#include "common.h"
#include "hal.h"
#include "log.h"
#include "mem/mem.h"

#define COMMAND_DATA_SIZE 7U

typedef uint8_t eFLIGHT_MODE_t;
enum { eFLIGHT_MODE_HOVER = 0, eFLIGHT_MODE_AIRPLANE };

typedef uint8_t eREQUESTED_STATE_t;
enum {
    /* These are the command states that a gui or radio controller can send */
    eREQUESTED_STATE_STOP = 0,
    eREQUESTED_STATE_START

    // eREQUESTED_STATE_SENSOR_ONLY
};

typedef uint8_t eOP_STATE_t;
enum {
    /* These are the states the FC can transition to based on the context and sent op state */
    eOP_STATE_STOPPED = 0,
    eOP_STATE_RUNNING,
    eOP_STATE_ERROR
};

typedef uint8_t eCOMMAND_t;
enum {
    eCOMMAND_TYPE_EMPTY = 0, // represents invalid command
    eCOMMAND_TYPE_CHANGE_OP_STATE,
    eCOMMAND_TYPE_CHANGE_FLIGHT_MODE,
    eCOMMAND_TYPE_CHANGE_VELOCITY,
    eCOMMAND_TYPE_CHANGE_PID
};

typedef int8_t velocity_t;
typedef uint16_t pid_t;
typedef uint8_t ePID_t;
enum { ePID_ROLL = 0, ePID_PITCH, ePID_YAW, ePID_THROTTLE };

typedef struct {
    eCOMMAND_t commandType;
} CommandHeader;

typedef struct {
    CommandHeader header;
    uint8_t data[COMMAND_DATA_SIZE];
} EmptyCommand;

typedef struct {
    CommandHeader header;
    eREQUESTED_STATE_t requestedState;
} ChangeOpStateCmd;

typedef struct {
    CommandHeader header;
    eFLIGHT_MODE_t flightMode;
} ChangeFlightModeCmd;

typedef struct {
    CommandHeader header;
    velocity_t vUp;      // -100 to 100
    velocity_t vRight;   // -100 to 100
    velocity_t vForward; // -100 to 100
} ChangeVelocityCmd;

typedef struct {
    CommandHeader header;
    ePID_t pidType;
    union {
        pid_t roll;
        pid_t pitch;
        pid_t yaw;
        pid_t throttle;
    } P; // 0 to 65,535

    union {
        pid_t roll;
        pid_t pitch;
        pid_t yaw;
        pid_t throttle;
    } I; // 0 to 65,535

    union {
        pid_t roll;
        pid_t pitch;
        pid_t yaw;
        pid_t throttle;
    } D; // 0 to 65,535
} ChangePIDCmd;

typedef struct {
    eFLIGHT_MODE_t flightMode;
    eOP_STATE_t opState;
} FCState;


STATIC_ASSERT (sizeof (EmptyCommand) <= MEM_SHARED_COMMAND_QUEUE_TOTAL_LEN, "");
STATIC_ASSERT (sizeof (ChangeOpStateCmd) <= sizeof (EmptyCommand), "");
STATIC_ASSERT (sizeof (ChangeFlightModeCmd) <= sizeof (EmptyCommand), "");
STATIC_ASSERT (sizeof (ChangeVelocityCmd) <= sizeof (EmptyCommand), "");
STATIC_ASSERT (sizeof (ChangePIDCmd) <= sizeof (EmptyCommand), "");
STATIC_ASSERT (sizeof (FCState) <= MEM_SHARED_FLIGHT_STATE_TOTAL_LEN, "");

eSTATUS_t ControlInit (void);
eSTATUS_t ControlStart (UART_HandleTypeDef* huart);
eSTATUS_t ControlProcessRawCmds (void);
BOOL_t ControlGetNewCmd (EmptyCommand* pOutCmd);
FCState ControlGetCopyFCState (void);
eOP_STATE_t ControlGetOpState (void);
eSTATUS_t ControlUpdateFCState (FCState const* pNewState);

#endif /* CONTROL_H */