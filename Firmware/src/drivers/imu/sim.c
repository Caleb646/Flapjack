
#include "drivers/imu/imudrv.h"

#include "core/core.h"

#include <string.h>
#include <stdbool.h>

typedef struct SimImu_s {
    uint8_t unused;
} SimImu_t;

STATIC eSTATUS_t Sim_Read (void* ctx, bool forcePolling, Vec3f* pAccel, Vec3f* pGyro) {

    FJ_UNUSED (forcePolling);
    SimImu_t* pSim = (SimImu_t*)ctx;
    if (!pSim || !pAccel || !pGyro) {
        return eSTATUS_FAILURE;
    }

    /* Level attitude: gravity on +Z, no rotation. */
    pAccel->x = 0.0F;
    pAccel->y = 0.0F;
    pAccel->z = 9.81F;
    pGyro->x  = 0.0F;
    pGyro->y  = 0.0F;
    pGyro->z  = 0.0F;
    return eSTATUS_SUCCESS;
}

STATIC bool Sim_IsDataReady (void* ctx) {
    // TODO
    SimImu_t* pSim = (SimImu_t*)ctx;
    if (!pSim) {
        return false;
    }
    return true;
}

// TODO: add to build system
// eSTATUS_t ImuDrv_Init (ImuDriverConf_t const* pConf, ImuDriver_t* pOutDriver) {

//     if (!pConf || !pOutDriver) {
//         return eSTATUS_NULL_ARG;
//     }

//     memset(pOutDriver, 0, sizeof(ImuDriver_t));
//     eSTATUS_t status = eSTATUS_SUCCESS;
//     do {
//         pOutDriver->ctx = Allocate(sizeof(SimImu_t));
//         if (!pOutDriver->ctx) {
//             status = eSTATUS_FAILURE;
//             break;
//         }

//         pOutDriver->Read = Sim_Read;
//         pOutDriver->IsDataReady = Sim_IsDataReady;

//     } while(0);
    
//     return status;
// }
