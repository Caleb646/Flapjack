#include "device/device.h"
#include "common.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "device/imu/imu.h"
#include "hal.h"
#include "log/logger.h"
#include "mc/actuators.h"
#include <stdint.h>
#include <string.h>

eSTATUS_t DeviceInitAll (BoardConf_t* pBoardConf) {

    if (pBoardConf == NULL) {
        LOG_ERROR ("Board configuration is NULL");
        return eSTATUS_FAILURE;
    }

    eSTATUS_t status = eSTATUS_SUCCESS;

    /*
     * Setup serial debug first so errors can be logged.
     * TODO: flash maybe should be setup first as well. So if serial debug
     * is not available logs can be written to flash.
     */
    DeviceBoardConf_t* pSerialDebugConf =
    BoardConfGetDeviceById (eSERIAL_DEBUG_DEVICE_ID);
    if (pSerialDebugConf != NULL) {
        SERIAL_DEBUG_INIT_FROM_BOARD_CONF (&status, *pSerialDebugConf);
        if (status != eSTATUS_SUCCESS) {
            return eSTATUS_FAILURE;
        }
    }

    for (uint32_t i = 0; i < pBoardConf->numDevices; ++i) {

        DeviceBoardConf_t deviceConf = pBoardConf->pDeviceBoardConfs[i];
        switch (deviceConf.deviceId) {
        case eIMU_DEVICE_ID:
            IMU_INIT (&status, deviceConf);
            LOG_ERROR_IF (status != eSTATUS_SUCCESS, "Failed to init IMU");
            break;
        case eMAG_DEVICE_ID: break;            // TODO
        case eBARO_DEVICE_ID: break;           // TODO
        case eGPS_DEVICE_ID: break;            // TODO
        case eFLASH_DEVICE_ID: break;          // TODO
        case eRF_RECEIVER_DEVICE_ID: break;    // TODO
        case eCURRENT_SENSOR_DEVICE_ID: break; // TODO
        default:
            LOG_ERROR ("Unknown device ID: %d", deviceConf.deviceId);
            continue;
        }
    }

    for (uint32_t i = 0; i < pBoardConf->numMotors; ++i) {

        MotorBoardConf_t motorBoardConf = pBoardConf->pMotorBoardConfs[i];
        ACTUATORS_INIT_MOTOR (&status, motorBoardConf);
        LOG_ERROR_IF (
        status != eSTATUS_SUCCESS,
        "Failed to init motor %d",
        motorBoardConf.motorId
        );
    }

    for (uint32_t i = 0; i < pBoardConf->numServos; ++i) {

        ServoBoardConf_t servoBoardConf = pBoardConf->pServoBoardConfs[i];
        ACTUATORS_INIT_SERVO (&status, servoBoardConf);
        LOG_ERROR_IF (
        status != eSTATUS_SUCCESS,
        "Failed to init servo %d",
        servoBoardConf.servoId
        );
    }

    return eSTATUS_SUCCESS;
}
