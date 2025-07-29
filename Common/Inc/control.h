#ifndef CONTROL_H
#define CONTROL_H

#include "common.h"
#include "hal.h"
#include "log.h"

typedef enum {
    eFLIGHT_MODE_HOVER = 0,
    eFLIGHT_MODE_AIRPLANE
} eFLIGHT_MODE;

typedef enum {
    eOP_STATE_START = 0,
    eOP_STATE_STOP,
    eOP_STATE_SENSOR_ONLY,
    eOP_STATE_RUNNING,
    eOP_STATE_IDLE
} eOP_STATE;

typedef enum {
    eCOMMAND_TYPE_CHANGE_OP_STATE = 0,
    eCOMMAND_TYPE_CHANGE_FLIGHT_MODE,
    eCOMMAND_TYPE_CHANGE_VEL,
    eCOMMAND_TYPE_CHANGE_PID
} eCOMMAND_TYPE;

typedef struct {
    eFLIGHT_MODE flightMode;
    eOP_STATE opState;
} FCState;

#endif /* CONTROL_H */