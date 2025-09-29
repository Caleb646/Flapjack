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
FCState_t FCStateGetCopyOfActiveState (void);

char const* OpState2Char (eOP_STATE_t opState);

#define FCSTATE_INIT(pSTATUS)                           \
    do {                                                \
        FCStateInitConf_t conf = { 0 };                 \
        *(pSTATUS)             = FCStateInit (conf, 0); \
    } while (0)


#endif // MC_FCSTATE_H