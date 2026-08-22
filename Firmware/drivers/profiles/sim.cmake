# Simulation (HIL) loadout: sensors and actuators are exchanged with the JSBSim
# bridge over the sim link instead of touching real hardware.
#
# The IMU is the exception - it stays on the real bmi323 driver. Under Renode
# the part itself is emulated (Scripts/renode/BMI323.cs) and the bridge pushes
# samples into that model, so a SIL run exercises the register map, the SPI
# framing and the chip-select timing instead of stepping over them. There is no
# sim IMU backend any more.
#
# The remaining sim backends call into drivers/sim_link. That module always compiles;
# SIM_HIL (added below) only gates its activation in main.c - starting the RX
# task and swapping the debug UART/shell for the binary sim link.
#
# RC is injected over the same link: sim_link writes g_Rx.channels directly, so
# rx stays on the (unused-in-sim) CRSF driver and needs no backend swap.
select_driver(imu   bmi323)
select_driver(mag   sim)
select_driver(baro  sim)
select_driver(servo sim)
select_driver(motor sim)

list(APPEND APP_DEFS "SIM_HIL")
