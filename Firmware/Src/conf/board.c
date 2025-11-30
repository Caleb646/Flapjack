#include "conf/board.h"
#include "common.h"

SHARED_MEM_SECTION DeviceTree_t g_DeviceTree = { 0 };

eSTATUS_t DeviceTree_Init (void) {

    eSTATUS_t status = DeviceTree_InitImpl (&g_DeviceTree);
    // #if defined(MY_BOARD)
    //     status = BoardConfInit_MyBoard (&g_BoardConf);
    // #elif defined(DEV_BOARD)
    //     status = BoardConfInit_DevBoard (&g_DeviceTree);
    // #else
    // #error "No board defined"
    // #endif

    if (FJ_OK (status)) {
        g_DeviceTree.isInitialized = true;
    }
    return status;
}

DeviceTree_t* DeviceTree_Get (void) {

    if (g_DeviceTree.isInitialized == false) {
        return NULL;
    }
    return &g_DeviceTree;
}

DevDesc_t* DeviceTree_GetDeviceById (eDEVICE_ID_t deviceId) {

    if (g_DeviceTree.isInitialized == false) {
        return NULL;
    }
    for (uint32_t i = 0; i < g_DeviceTree.numDevices; ++i) {
        if (g_DeviceTree.ppDeviceDescs[i]->deviceId == deviceId) {
            return g_DeviceTree.ppDeviceDescs[i];
        }
    }
    return NULL;
}