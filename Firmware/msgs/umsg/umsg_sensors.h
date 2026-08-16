// Generated with umsg_gen - do not edit by hand
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <umsg_types.h>

// msg structure typedefs
typedef struct
{
    float gyro[3];
    float accel[3];
    float temperature;
} umsg_sensors_imu_t;

typedef struct
{
    float field[3];
} umsg_sensors_mag_t;

typedef struct
{
    float pressure;
    float temperature;
} umsg_sensors_baro_t;

typedef struct
{
    double lat;
    double lon;
    float alt;
    float speed;
    float course;
    uint8_t fix_type;
    uint8_t sats;
} umsg_sensors_gps_t;

// api function headers
umsg_sub_handle_t umsg_sensors_imu_subscribe(uint32_t prescaler, uint8_t length);
umsg_sub_handle_t umsg_sensors_imu_subscribe_ch(uint32_t prescaler, uint8_t length, uint8_t channel);
void umsg_sensors_imu_publish(umsg_sensors_imu_t* data);
void umsg_sensors_imu_publish_ch(umsg_sensors_imu_t* data, uint8_t channel);
uint8_t umsg_sensors_imu_receive(umsg_sub_handle_t queue, umsg_sensors_imu_t* data, uint32_t timeout);

umsg_sub_handle_t umsg_sensors_mag_subscribe(uint32_t prescaler, uint8_t length);
umsg_sub_handle_t umsg_sensors_mag_subscribe_ch(uint32_t prescaler, uint8_t length, uint8_t channel);
void umsg_sensors_mag_publish(umsg_sensors_mag_t* data);
void umsg_sensors_mag_publish_ch(umsg_sensors_mag_t* data, uint8_t channel);
uint8_t umsg_sensors_mag_receive(umsg_sub_handle_t queue, umsg_sensors_mag_t* data, uint32_t timeout);

umsg_sub_handle_t umsg_sensors_baro_subscribe(uint32_t prescaler, uint8_t length);
umsg_sub_handle_t umsg_sensors_baro_subscribe_ch(uint32_t prescaler, uint8_t length, uint8_t channel);
void umsg_sensors_baro_publish(umsg_sensors_baro_t* data);
void umsg_sensors_baro_publish_ch(umsg_sensors_baro_t* data, uint8_t channel);
uint8_t umsg_sensors_baro_receive(umsg_sub_handle_t queue, umsg_sensors_baro_t* data, uint32_t timeout);

umsg_sub_handle_t umsg_sensors_gps_subscribe(uint32_t prescaler, uint8_t length);
umsg_sub_handle_t umsg_sensors_gps_subscribe_ch(uint32_t prescaler, uint8_t length, uint8_t channel);
void umsg_sensors_gps_publish(umsg_sensors_gps_t* data);
void umsg_sensors_gps_publish_ch(umsg_sensors_gps_t* data, uint8_t channel);
uint8_t umsg_sensors_gps_receive(umsg_sub_handle_t queue, umsg_sensors_gps_t* data, uint32_t timeout);


#ifdef __cplusplus
}
#endif

