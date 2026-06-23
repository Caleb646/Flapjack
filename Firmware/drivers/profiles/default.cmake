# Default hardware loadout: real sensor chips.
select_driver(imu bmi323)
select_driver(mag mmc5983)
# select_driver(gps   nmea)    # when gps adopts the driver-selection pattern
# select_driver(motor pwm)     # when actuators adopt it
