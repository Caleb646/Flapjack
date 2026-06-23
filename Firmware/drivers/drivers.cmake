# Compile-time driver selection. The root CMakeLists.txt just include()s this.
#
# A driver profile (Firmware/drivers/profiles/<profile>.cmake) calls
# select_driver() once per interface to pick a backend. Every driver source
# compiles unless pruned by a select_driver() call.
#
# Convention: selectable backends for interface <iface> are the top-level .c
# files in drivers/<iface>/. Shared helpers (subdirectories, and top-level files
# like dma.c/timer.c) always compile.

set(DRIVERS_ROOT "${CMAKE_CURRENT_LIST_DIR}")   # capture at FILE scope (see note)

# Every driver source compiles unless pruned by a select_driver() below.
file(GLOB_RECURSE DRIVER_SRCS "${DRIVERS_ROOT}/*.c")

# Pick one backend for an interface: drop that interface's other top-level .c
# files and keep only the chosen one.
macro(select_driver iface backend)
    set(_dir "${DRIVERS_ROOT}/${iface}")
    file(GLOB _all "${_dir}/*.c")          # top-level only; subdir helpers stay
    list(REMOVE_ITEM DRIVER_SRCS ${_all})
    set(_chosen "${_dir}/${backend}.c")
    if(NOT EXISTS "${_chosen}")
        message(FATAL_ERROR "Driver '${backend}' not found for '${iface}' (${_chosen})")
    endif()
    list(APPEND DRIVER_SRCS "${_chosen}")
    message(STATUS "Driver [${iface}] = ${backend}")
endmacro()

# NOTE: DRIVERS_ROOT is captured at file scope above. Do NOT use
# CMAKE_CURRENT_LIST_DIR inside select_driver — macros expand at the call site
# (profiles/<name>.cmake), where it would resolve to the profiles/ folder.

if(NOT DEFINED DRIVER_PROFILE)
    set(DRIVER_PROFILE default)
endif()
include("${DRIVERS_ROOT}/profiles/${DRIVER_PROFILE}.cmake")
message(STATUS "Driver profile: ${DRIVER_PROFILE}")

list(APPEND APP_SRCS ${DRIVER_SRCS})        # in-scope: included into root
