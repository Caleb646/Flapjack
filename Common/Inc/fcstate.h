#ifndef MC_FCSTATE_H
#define MC_FCSTATE_H

#include "common.h"
#include <stdbool.h>
#include <stdint.h>

typedef uint8_t eFLIGHT_MODE_t;
enum { eFLIGHT_MODE_HOVER = 0, eFLIGHT_MODE_AIRPLANE };

typedef uint8_t eOP_STATE_t;
enum {
    eOP_STATE_STOPPED = 0,
    eOP_STATE_RUNNING,
    eOP_STATE_ERROR,

    eNUMBER_OF_OP_STATES
};

typedef struct {
    uint8_t unused;
} FCStateInitConf_t;

typedef struct {
    eFLIGHT_MODE_t flightMode;
    eOP_STATE_t opState;
    Vec3f currentAttitude; // degrees
    Vec3f targetAttitude;  // degrees
    Vec3f maxAttitude;     // degrees
    float targetThrottle;  // 0.0 to 1.0
    bool isInitialized;
} FCState_t;


// typedef FCState volatile vFCState;
typedef FCState_t vFCState_t;

eSTATUS_t FCStateInit (FCStateInitConf_t conf, vFCState_t* pOut);
eSTATUS_t FCStateStart (vFCState_t* pState);
eSTATUS_t FCStateStop (vFCState_t* pState);
vFCState_t const* FCStateGetActiveState (void);
vFCState_t* FCStateGetMutableActiveState (void);
FCState_t FCStateGetCopyOfActiveState (void);
bool FCStateSetOpState (vFCState_t* pState, eOP_STATE_t newOpState);

bool FCStateSetCurrentAttitude (vFCState_t* pState, Vec3f newAttitude);
bool FCStateSetTargetAttitude (vFCState_t* pState, Vec3f newAttitude);
bool FCStateSetMaxAttitude (vFCState_t* pState, Vec3f newAttitude);
bool FCStateSetTargetThrottle (vFCState_t* pState, float newThrottle);

char const* OpState2Char (eOP_STATE_t opState);

#define FCSTATE_INIT(pSTATUS)                           \
    do {                                                \
        FCStateInitConf_t conf = { 0 };                 \
        *(pSTATUS)             = FCStateInit (conf, 0); \
    } while (0)

// clang-format off

#define FC_SET_STOPPED_OP_STATE() FCStateSetOpState (FCStateGetMutableActiveState (), eOP_STATE_STOPPED)
#define FC_SET_RUNNING_OP_STATE() FCStateSetOpState (FCStateGetMutableActiveState (), eOP_STATE_RUNNING)
#define FC_SET_ERROR_OP_STATE() FCStateSetOpState (FCStateGetMutableActiveState (), eOP_STATE_ERROR)

// clang-format on

#endif // MC_FCSTATE_H