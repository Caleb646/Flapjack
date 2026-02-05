#ifndef MC_FCSTATE_H
#define MC_FCSTATE_H

#include "core/core.h"
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

eSTATUS_t FCState_Init (FCStateInitConf_t conf, vFCState_t* pOut);
eSTATUS_t FCState_Start (vFCState_t* pState);
eSTATUS_t FCState_Stop (vFCState_t* pState);
vFCState_t const* FCState_GetActiveState (void);
vFCState_t* FCState_GetMutableActiveState (void);
FCState_t FCState_GetCopyOfActiveState (void);
bool FCState_Set_OpState (vFCState_t* pState, eOP_STATE_t newOpState);
bool FCState_Set_CurrentAttitude (vFCState_t* pState, Vec3f newCurrentAttitude);
bool FCState_Set_TargetAttitude (vFCState_t* pState, Vec3f newTargetAttitude);
bool FCState_Set_MaxAttitude (vFCState_t* pState, Vec3f newMaxAttitude);
bool FCState_Set_TargetThrottle (vFCState_t* pState, float newThrottle);

char const* OpStateToChar (eOP_STATE_t opState);

#define FCSTATE_INIT(pSTATUS)                            \
    do {                                                 \
        FCStateInitConf_t conf = { 0 };                  \
        *(pSTATUS)             = FCState_Init (conf, 0); \
    } while (0)

// clang-format off

#define FC_SET_STOPPED_OP_STATE() FCState_Set_OpState (FCState_GetMutableActiveState (), eOP_STATE_STOPPED)
#define FC_SET_RUNNING_OP_STATE() FCState_Set_OpState (FCState_GetMutableActiveState (), eOP_STATE_RUNNING)
#define FC_SET_ERROR_OP_STATE() FCState_Set_OpState (FCState_GetMutableActiveState (), eOP_STATE_ERROR)

#define FC_SET_CURRENT_ATTITUDE(NEW_CURRENT_ATTITUDE) FCState_Set_CurrentAttitude (FCState_GetMutableActiveState (), (NEW_CURRENT_ATTITUDE))

// clang-format on

#endif // MC_FCSTATE_H