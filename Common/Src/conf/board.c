#include "conf/board.h"

bool ge_isBoardConfInitialized = false;
BoardConf_t ge_BoardConf       = { 0 };

BoardConf_t* BoardConfGet (void) {

    if (ge_isBoardConfInitialized == false) {
        return 0;
    }
    return &ge_BoardConf;
}

DeviceBoardConf_t* BoardConfGetDeviceById (eDEVICE_ID_t deviceId) {

    if (ge_isBoardConfInitialized == false) {
        return 0;
    }
    for (uint32_t i = 0; i < ge_BoardConf.numDevices; ++i) {
        if (ge_BoardConf.ppDeviceBoardConfs[i]->deviceId == deviceId) {
            return ge_BoardConf.ppDeviceBoardConfs[i];
        }
    }
    return 0;
}