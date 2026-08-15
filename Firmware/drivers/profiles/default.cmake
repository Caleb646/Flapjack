# Default hardware loadout: real sensor chips and actuators.
select_driver(imu   bmi323)
select_driver(mag   mmc5983)
select_driver(servo pwm)
select_driver(motor dshot)
# select_driver(gps   nmea)    # when gps adopts the driver-selection pattern
# rx stays on the CRSF driver (rx.c + crsf.c helper); not a select_driver iface.
