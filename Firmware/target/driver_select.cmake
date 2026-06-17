# Compile-time driver selection.
#
# A driver profile (Firmware/target/drivers/<profile>.cmake) calls select_driver()
# once per interface to pick a backend. apply_drivers() then prunes APP_SRCS so
# only the selected backend compiles for each interface.
#
# Convention: selectable backends for interface <iface> are the top-level .c files
# in Firmware/drivers/<iface>/. Shared helpers live in subdirectories and
# always compile.

set(DRIVER_IFACES "")

# Record one interface->backend choice (called from a profile file).
macro(select_driver iface backend)
    list(APPEND DRIVER_IFACES "${iface}")
    set(DRIVER_${iface} "${backend}")
endmacro()

# Prune APP_SRCS so only the selected backend compiles per interface.
macro(apply_drivers)
    foreach(iface IN LISTS DRIVER_IFACES)
        set(_dir "${FIRMWARE_ROOT}/drivers/${iface}")
        file(GLOB _all "${_dir}/*.c")          # top-level only; helpers in subdirs stay
        list(REMOVE_ITEM APP_SRCS ${_all})
        set(_chosen "${_dir}/${DRIVER_${iface}}.c")
        if(NOT EXISTS "${_chosen}")
            message(FATAL_ERROR "Driver '${DRIVER_${iface}}' not found for '${iface}' (${_chosen})")
        endif()
        list(APPEND APP_SRCS "${_chosen}")
        message(STATUS "Driver [${iface}] = ${DRIVER_${iface}}")
    endforeach()
endmacro()
