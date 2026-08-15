# Simulation (HIL) loadout: sensors and actuators are exchanged with the JSBSim
# bridge over the sim link instead of touching real hardware.
#
# The sim backends call into drivers/sim_link. That module always compiles;
# SIM_HIL (added below) only gates its activation in main.c - starting the RX
# task and swapping the debug UART/shell for the binary sim link.
#
# RC is injected over the same link: sim_link writes g_Rx.channels directly, so
# rx stays on the (unused-in-sim) CRSF driver and needs no backend swap.
select_driver(imu   sim)
select_driver(mag   sim)
select_driver(servo sim)
select_driver(motor sim)

list(APPEND APP_DEFS "SIM_HIL")
