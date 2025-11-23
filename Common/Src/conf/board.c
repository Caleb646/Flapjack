#include "conf/board.h"
#include "common.h"

SHARED_MEM_SECTION BoardConf_t g_BoardConf = { 0 };

eSTATUS_t BoardConfInit (void) {

    eSTATUS_t status = eSTATUS_SUCCESS;
#if defined(MY_BOARD)
    status = BoardConfInit_MyBoard ();
#elif defined(DEV_BOARD)
    status = BoardConfInit_DevBoard ();
#else
#error "No board defined"
#endif

    if (FJ_OK (status)) {
        g_BoardConf.isInitialized = true;
    }
    return status;
}

BoardConf_t* BoardConfGet (void) {

    if (g_BoardConf.isInitialized == false) {
        return 0;
    }
    return &g_BoardConf;
}

DeviceBoardConf_t* BoardConfGetDeviceById (eDEVICE_ID_t deviceId) {

    if (g_BoardConf.isInitialized == false) {
        return 0;
    }
    for (uint32_t i = 0; i < g_BoardConf.numDevices; ++i) {
        if (g_BoardConf.ppDeviceBoardConfs[i]->deviceId == deviceId) {
            return g_BoardConf.ppDeviceBoardConfs[i];
        }
    }
    return 0;
}