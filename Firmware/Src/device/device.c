#include "device/device.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "conf/ids.h"
#include "core/core.h"
#include "core/log/logger.h"
#include "device/flash/flash.h"
#include "device/gps/gps.h"
#include "device/imu/imu.h"
#include "device/mag/mag.h"
#include "device/motor/motor.h"
#include "device/serial/serial.h"
#include "device/servo/servo.h"
#include "hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>


eSTATUS_t Device_InitAll (BoardConf_t* pBoardConf) {

    RETURN_IF_NULL (pBoardConf, eSTATUS_FAILURE, "Board configuration is NULL");
    eSTATUS_t status = eSTATUS_SUCCESS;
    /*
     * Setup serial debug first so errors can be logged.
     * TODO: flash maybe should be setup first as well. So if serial debug
     * is not available logs can be written to flash.
     */
    DeviceBoardConf_t* pSerialDebugConf = BoardConfGetDeviceById (eSERIAL_DEBUG_DEVICE_ID);
    if (pSerialDebugConf != NULL) {
        SERIAL_DEBUG_INIT (&status, *pSerialDebugConf);
        RETURN_IF (FJ_FAIL (status), status, "Failed to init serial debug");
    }

    for (uint32_t i = 0; i < pBoardConf->numDevices; ++i) {

        DeviceBoardConf_t* pDevConf = pBoardConf->ppDeviceBoardConfs[i];
        eDEVICE_ID_t deviceId       = pDevConf->deviceId;
        switch (deviceId) {
        case eIMU_DEVICE_ID:
            IMU_INIT (&status, *pDevConf);
            LOG_ERROR_IF (status != eSTATUS_SUCCESS, "Failed to init IMU");
            break;
        case eMAG_DEVICE_ID: break;            // TODO
        case eBARO_DEVICE_ID: break;           // TODO
        case eGPS_DEVICE_ID: break;            // TODO
        case eFLASH_DEVICE_ID: break;          // TODO
        case eRF_RECEIVER_DEVICE_ID: break;    // TODO
        case eCURRENT_SENSOR_DEVICE_ID: break; // TODO
        default:
            if (DEVICE_ID_IS_MOTOR (deviceId) == true) {
                MOTOR_INIT (&status, *pDevConf);
                RETURN_IF (FJ_FAIL (status), status, "Failed to init motor %d", deviceId);
                break;
            }
            if (DEVICE_ID_IS_SERVO (deviceId) == true) {
                SERVO_INIT (&status, *pDevConf);
                RETURN_IF (FJ_FAIL (status), status, "Failed to init servo %d", deviceId);
                break;
            }
        }
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t Device_StartAll (void) {

    eSTATUS_t status = eSTATUS_SUCCESS;

    // Serial Debug is allowed to fail
    status = SerialDebugStart (SerialDebugGetMutableActiveDevice ());
    // Flash is allowed to fail
    status = FlashStart (FlashGetMutableActiveDevice ());
    LOG_ERROR_IF (FJ_FAIL (status), "Failed to start Flash");

    // IMU is required
    status = IMUStart (IMU_GetMutableActiveDevice ());
    RETURN_IF (FJ_FAIL (status), status, "Failed to start IMU");

    // Mag is allowed to fail
    status = MagStart (Mag_GetMutableActiveDevice ());
    LOG_ERROR_IF (FJ_FAIL (status), "Failed to start Magnetometer");

    // GPS is allowed to fail
    status = GPSStart (GPSGetMutableActiveDevice ());
    LOG_ERROR_IF (FJ_FAIL (status), "Failed to start GPS");

    // NOTE: Leave motors and servos to be started by the motion control (mc) module

    return eSTATUS_SUCCESS;
}
