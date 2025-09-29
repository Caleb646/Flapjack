
#include "mc/fcstate.h"
#include "common.h"
#include "conf/conf.h"
#include "mem/mem.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define FCSTATE_VALID(pSTATE) \
    ((pSTATE) != NULL && (pSTATE)->isInitialized == true)

static SHARED_MEM_SECTION FCState_t gFCState = { 0 };

eSTATUS_t FCStateInit (FCStateInitConf_t conf, vFCState_t* pOut) {

    vFCState_t* pState = &gFCState;
    if (pOut != NULL) {
        pState = pOut;
    }

    memset ((void*)pState, 0, sizeof (vFCState_t));
    pState->flightMode      = eFLIGHT_MODE_HOVER;
    pState->opState         = eOP_STATE_STOPPED;
    pState->currentAttitude = (Vec3f){ 0.0F, 0.0F, 0.0F };
    pState->targetAttitude  = (Vec3f){ 0.0F, 0.0F, 0.0F };
    pState->maxAttitude     = (Vec3f){ 45.0F, 45.0F, 180.0F };
    pState->targetThrottle  = MOTOR_STARTUP_THROTTLE; // between 0 and 1
    pState->isInitialized   = true;
    return eSTATUS_SUCCESS;
}

eSTATUS_t FCStateStart (vFCState_t* pState) {

    if (FCSTATE_VALID (pState) == false) {
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t FCStateStop (vFCState_t* pState) {

    if (FCSTATE_VALID (pState) == false) {
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t FCStateUpdate (vFCState_t const* pNewState, vFCState_t* pOutState) {

    if (FCSTATE_VALID (pNewState) == false) {
        return eSTATUS_FAILURE;
    }

    if (pOutState == NULL) {
        pOutState = &gFCState;
    }

    *pOutState = *pNewState;
    return eSTATUS_SUCCESS;
}

vFCState_t const* FCStateGetActiveState (void) {

    if (FCSTATE_VALID (&gFCState) == false) {
        return NULL;
    }
    return &gFCState;
}

FCState_t FCStateGetCopyOfActiveState (void) {

    if (FCSTATE_VALID (&gFCState) == false) {
        return (FCState_t){ 0 };
    }
    return gFCState;
}

char const* OpState2Char (eOP_STATE_t opState) {

    switch (opState) {
    case eOP_STATE_STOPPED: return "[STOPPED]";
    case eOP_STATE_RUNNING: return "[RUNNING]";
    case eOP_STATE_ERROR: return "[ERROR]";
    default: return "[UNKNOWN]";
    }
}