#include "core/core.h"

eSTATUS_t Core_Init (void) {

    if (CoreShared_Init () != eSTATUS_SUCCESS) {
        return eSTATUS_FAILURE;
    }

    if (SyncInit () != eSTATUS_SUCCESS) {
        return eSTATUS_FAILURE;
    }

    if (LoggerInit () != eSTATUS_SUCCESS) {
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}