// Generated with umsg_gen on 2026-08-15
#include <umsg.h>
#include <umsg_sensors.h>

// msg instances
static umsg_msg_metadata_t msg_sensors_imu = {.name = "sensors_imu"};
static umsg_msg_metadata_t msg_sensors_mag = {.name = "sensors_mag"};
static umsg_msg_metadata_t msg_sensors_baro = {.name = "sensors_baro"};
static umsg_msg_metadata_t msg_sensors_gps = {.name = "sensors_gps"};

// msg api's
// sensors_imu
umsg_sub_handle_t umsg_sensors_imu_subscribe(uint32_t prescaler, uint8_t length)
{
    return umsg_subscribe(&msg_sensors_imu, prescaler, sizeof(umsg_sensors_imu_t), length, 0);
}
umsg_sub_handle_t umsg_sensors_imu_subscribe_ch(uint32_t prescaler, uint8_t length, uint8_t channel)
{
    return umsg_subscribe(&msg_sensors_imu, prescaler, sizeof(umsg_sensors_imu_t), length, channel);
}
void umsg_sensors_imu_publish(umsg_sensors_imu_t* data)
{
    umsg_publish(&msg_sensors_imu, data, 0);
}
void umsg_sensors_imu_publish_ch(umsg_sensors_imu_t* data, uint8_t channel)
{
    umsg_publish(&msg_sensors_imu, data, channel);
}
uint8_t umsg_sensors_imu_receive(umsg_sub_handle_t queue, umsg_sensors_imu_t* data, uint32_t timeout)
{
    return umsg_receive(queue, data, timeout);
}
// sensors_mag
umsg_sub_handle_t umsg_sensors_mag_subscribe(uint32_t prescaler, uint8_t length)
{
    return umsg_subscribe(&msg_sensors_mag, prescaler, sizeof(umsg_sensors_mag_t), length, 0);
}
umsg_sub_handle_t umsg_sensors_mag_subscribe_ch(uint32_t prescaler, uint8_t length, uint8_t channel)
{
    return umsg_subscribe(&msg_sensors_mag, prescaler, sizeof(umsg_sensors_mag_t), length, channel);
}
void umsg_sensors_mag_publish(umsg_sensors_mag_t* data)
{
    umsg_publish(&msg_sensors_mag, data, 0);
}
void umsg_sensors_mag_publish_ch(umsg_sensors_mag_t* data, uint8_t channel)
{
    umsg_publish(&msg_sensors_mag, data, channel);
}
uint8_t umsg_sensors_mag_receive(umsg_sub_handle_t queue, umsg_sensors_mag_t* data, uint32_t timeout)
{
    return umsg_receive(queue, data, timeout);
}
// sensors_baro
umsg_sub_handle_t umsg_sensors_baro_subscribe(uint32_t prescaler, uint8_t length)
{
    return umsg_subscribe(&msg_sensors_baro, prescaler, sizeof(umsg_sensors_baro_t), length, 0);
}
umsg_sub_handle_t umsg_sensors_baro_subscribe_ch(uint32_t prescaler, uint8_t length, uint8_t channel)
{
    return umsg_subscribe(&msg_sensors_baro, prescaler, sizeof(umsg_sensors_baro_t), length, channel);
}
void umsg_sensors_baro_publish(umsg_sensors_baro_t* data)
{
    umsg_publish(&msg_sensors_baro, data, 0);
}
void umsg_sensors_baro_publish_ch(umsg_sensors_baro_t* data, uint8_t channel)
{
    umsg_publish(&msg_sensors_baro, data, channel);
}
uint8_t umsg_sensors_baro_receive(umsg_sub_handle_t queue, umsg_sensors_baro_t* data, uint32_t timeout)
{
    return umsg_receive(queue, data, timeout);
}
// sensors_gps
umsg_sub_handle_t umsg_sensors_gps_subscribe(uint32_t prescaler, uint8_t length)
{
    return umsg_subscribe(&msg_sensors_gps, prescaler, sizeof(umsg_sensors_gps_t), length, 0);
}
umsg_sub_handle_t umsg_sensors_gps_subscribe_ch(uint32_t prescaler, uint8_t length, uint8_t channel)
{
    return umsg_subscribe(&msg_sensors_gps, prescaler, sizeof(umsg_sensors_gps_t), length, channel);
}
void umsg_sensors_gps_publish(umsg_sensors_gps_t* data)
{
    umsg_publish(&msg_sensors_gps, data, 0);
}
void umsg_sensors_gps_publish_ch(umsg_sensors_gps_t* data, uint8_t channel)
{
    umsg_publish(&msg_sensors_gps, data, channel);
}
uint8_t umsg_sensors_gps_receive(umsg_sub_handle_t queue, umsg_sensors_gps_t* data, uint32_t timeout)
{
    return umsg_receive(queue, data, timeout);
}
