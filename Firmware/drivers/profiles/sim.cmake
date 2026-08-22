# Simulation (HIL) loadout: the actuators are exchanged with the JSBSim bridge
# over the sim link instead of driving real hardware.
#
# The SENSORS are not. All three run their real drivers: under Renode the parts
# themselves are emulated (Scripts/renode/{BMI323,MMC5983,BMP390}.cs) and the
# bridge pushes samples into those models, so a SIL run exercises the register
# maps, the SPI framing, the chip-select timing and - for the baro - the
# calibration decode and compensation polynomial, instead of stepping over all
# of it. There are no sim sensor backends any more.
#
# The remaining sim backends call into drivers/sim_link. That module always compiles;
# SIM_HIL (added below) only gates its activation in main.c - starting the RX
# task and swapping the debug UART/shell for the binary sim link.
#
# RC is injected over the same link: sim_link writes g_Rx.channels directly, so
# rx stays on the (unused-in-sim) CRSF driver and needs no backend swap.
select_driver(imu   bmi323)
select_driver(mag   mmc5983)
select_driver(baro  bmp390)
select_driver(servo sim)
select_driver(motor sim)

list(APPEND APP_DEFS "SIM_HIL")
