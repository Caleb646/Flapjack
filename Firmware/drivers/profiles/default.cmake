# Default hardware loadout: real sensor chips and actuators.
select_driver(imu   bmi323)
select_driver(mag   mmc5983)
# BMP390 on SPI5, sharing the bus with the magnetometer (separate NSS: mag on
# GPIOF pin 4, baro on pin 6 - see target/flapjack_v1/flapjack_v1.h).
#
# This must be selected rather than left out: without a select_driver() call
# every top-level .c in drivers/baro/ compiles, which would link the sim backend
# into flight firmware.
select_driver(baro  bmp390)
select_driver(servo pwm)
select_driver(motor dshot)
# select_driver(gps   nmea)    # when gps adopts the driver-selection pattern
# rx stays on the CRSF driver (rx.c + crsf.c helper); not a select_driver iface.
