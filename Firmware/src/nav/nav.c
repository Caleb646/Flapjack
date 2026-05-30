#include "nav/nav.h"
#include "mc/filter.h"
#include "umsg_sensors.h"
#include "umsg_nav.h"

#include "FreeRTOS.h"

typedef struct {
    MadgwickFilter_t filter;
    umsg_sub_handle_t imu_sub;
    umsg_sub_handle_t mag_sub;
    uint32_t usLastUpdateTime;
} Nav_t;

static Nav_t s_Nav;

eSTATUS_t Nav_Init(void) {
    s_Nav.imu_sub          = umsg_sensors_imu_subscribe(1, 4);
    s_Nav.mag_sub          = umsg_sensors_mag_subscribe(1, 4);
    s_Nav.usLastUpdateTime = GetMicroseconds();
    return MadgwickFilter_Init(&s_Nav.filter);
}

eSTATUS_t Nav_Update(void) {
    umsg_sensors_imu_t imu_msg;
    if (!umsg_sensors_imu_receive(s_Nav.imu_sub, &imu_msg, portMAX_DELAY)) {
        return eSTATUS_FAILURE;
    }

    uint32_t usNow = GetMicroseconds();
    float dt = (float)(usNow - s_Nav.usLastUpdateTime) / 1000000.0f;
    s_Nav.usLastUpdateTime = usNow;

    Vec3f accel;
    accel.x = imu_msg.accel[0];
    accel.y = imu_msg.accel[1];
    accel.z = imu_msg.accel[2];

    Vec3f gyro;
    gyro.x = imu_msg.gyro[0];
    gyro.y = imu_msg.gyro[1];
    gyro.z = imu_msg.gyro[2];

    umsg_sensors_mag_t mag_msg;
    Vec3f mag;
    Vec3f* pMag = NULL;
    if (umsg_sensors_mag_receive(s_Nav.mag_sub, &mag_msg, 0)) {
        mag.x = mag_msg.field[0];
        mag.y = mag_msg.field[1];
        mag.z = mag_msg.field[2];
        pMag  = &mag;
    }

    Vec3f euler;
    eSTATUS_t status = MadgwickFilter_Update(&s_Nav.filter, &accel, &gyro, pMag, dt, &euler);
    if (STATUS_FAIL(status)) {
        return status;
    }

    umsg_nav_state_t msg;
    msg.quat[0]    = s_Nav.filter.qEst.q1;
    msg.quat[1]    = s_Nav.filter.qEst.q2;
    msg.quat[2]    = s_Nav.filter.qEst.q3;
    msg.quat[3]    = s_Nav.filter.qEst.q4;
    msg.euler[0]   = euler.x;
    msg.euler[1]   = euler.y;
    msg.euler[2]   = euler.z;
    msg.gyro[0]    = imu_msg.gyro[0];
    msg.gyro[1]    = imu_msg.gyro[1];
    msg.gyro[2]    = imu_msg.gyro[2];
    msg.pos_ned[0] = 0.0f;
    msg.pos_ned[1] = 0.0f;
    msg.pos_ned[2] = 0.0f;
    msg.vel_ned[0] = 0.0f;
    msg.vel_ned[1] = 0.0f;
    msg.vel_ned[2] = 0.0f;
    msg.alt        = 0.0f;
    msg.valid      = NAV_VALID_ATTITUDE;
    umsg_nav_state_publish(&msg);
    return eSTATUS_SUCCESS;
}
