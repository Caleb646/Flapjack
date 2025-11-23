
#include "fcstate.h"
#include "common.h"
#include "conf/conf.h"
#include "core/core.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>



#define FCSTATE_VALID(pSTATE) ((pSTATE) != NULL && (pSTATE)->isInitialized == true)

static SHARED_MEM_SECTION FCState_t gFCState = { 0 };

eSTATUS_t FCState_Init (FCStateInitConf_t conf, vFCState_t* pOut) {

    FJ_UNUSED (conf);

    vFCState_t* pState = &gFCState;
    if (pOut != NULL) {
        pState = pOut;
    }

    if (pState->isInitialized == true) {
        return eSTATUS_ALREADY_INITED;
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

eSTATUS_t FCState_Start (vFCState_t* pState) {

    if (FCSTATE_VALID (pState) == false) {
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t FCState_Stop (vFCState_t* pState) {

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

vFCState_t const* FCState_GetActiveState (void) {

    if (FCSTATE_VALID (&gFCState) == false) {
        return NULL;
    }
    return &gFCState;
}

vFCState_t* FCState_GetMutableActiveState (void) {

    if (FCSTATE_VALID (&gFCState) == false) {
        return NULL;
    }
    return &gFCState;
}

FCState_t FCState_GetCopyOfActiveState (void) {

    if (FCSTATE_VALID (&gFCState) == false) {
        return (FCState_t){ 0 };
    }
    return gFCState;
}

bool FCState_Set_OpState (vFCState_t* pState, eOP_STATE_t newOpState) {

    if (FCSTATE_VALID (pState) == false) {
        return false;
    }
    pState->opState = newOpState;
    return true;
}

bool FCState_Set_CurrentAttitude (vFCState_t* pState, Vec3f newCurrentAttitude) {

    if (FCSTATE_VALID (pState) == false) {
        return false;
    }
    pState->currentAttitude = newCurrentAttitude;
    return true;
}

bool FCState_Set_TargetAttitude (vFCState_t* pState, Vec3f newTargetAttitude) {

    if (FCSTATE_VALID (pState) == false) {
        return false;
    }
    pState->targetAttitude = newTargetAttitude;
    return true;
}

bool FCState_Set_MaxAttitude (vFCState_t* pState, Vec3f newMaxAttitude) {

    if (FCSTATE_VALID (pState) == false) {
        return false;
    }
    pState->maxAttitude = newMaxAttitude;
    return true;
}

bool FCState_Set_TargetThrottle (vFCState_t* pState, float newThrottle) {

    if (FCSTATE_VALID (pState) == false) {
        return false;
    }
    if (newThrottle < 0.0F || newThrottle > 1.0F) {
        return false;
    }
    pState->targetThrottle = newThrottle;
    return true;
}

char const* OpStateToChar (eOP_STATE_t opState) {

    switch (opState) {
    case eOP_STATE_STOPPED: return "[STOPPED]";
    case eOP_STATE_RUNNING: return "[RUNNING]";
    case eOP_STATE_ERROR: return "[ERROR]";
    default: return "[UNKNOWN]";
    }
}