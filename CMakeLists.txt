cmake_minimum_required(VERSION 3.20)

######## Toolchain setup start ###########

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(FLAPJACK_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(TOOLCHAIN_BIN_PATH "${FLAPJACK_ROOT}/Tools/arm-gnu/bin")

find_program(CMAKE_C_COMPILER arm-none-eabi-gcc HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_CXX_COMPILER arm-none-eabi-g++ HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_ASM_COMPILER arm-none-eabi-gcc HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_AR arm-none-eabi-ar HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_OBJDUMP arm-none-eabi-objdump HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_SIZE arm-none-eabi-size HINTS ${TOOLCHAIN_BIN_PATH})

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_EXECUTABLE_SUFFIX_C   .elf)
set(CMAKE_EXECUTABLE_SUFFIX_CXX .elf)
set(CMAKE_EXECUTABLE_SUFFIX_ASM .elf)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

######## Toolchain setup end ###########

project(FlapJack C ASM)

set(TESTS_ROOT "${FLAPJACK_ROOT}/Tests")
set(FIRMWARE_ROOT "${FLAPJACK_ROOT}/Firmware")
set(PLATFORM_ROOT "${FIRMWARE_ROOT}/platform")
set(TARGET_ROOT "${FIRMWARE_ROOT}/target")

# ── Vendor group ─────────────────────────────────────────────────────────────
set(NANOPB_ROOT    "${FLAPJACK_ROOT}/Vendor/nanopb")
set(FREERTOS_ROOT  "${FLAPJACK_ROOT}/Vendor/FreeRTOS/Source")
set(UMSG_ROOT      "${FLAPJACK_ROOT}/Vendor/umsg/umsg_lib/core")

set(GENERATED_MSG_INCS "${FIRMWARE_ROOT}/msgs/proto" "${FIRMWARE_ROOT}/msgs/umsg")
file(GLOB_RECURSE GENERATED_MSG_SRCS "${FIRMWARE_ROOT}/msgs/*.c")

set(VENDOR_SRCS
    # nanopb
    "${NANOPB_ROOT}/pb_common.c"
    "${NANOPB_ROOT}/pb_encode.c"
    "${NANOPB_ROOT}/pb_decode.c"
    # FreeRTOS kernel (portable layer is per-core)
    "${FREERTOS_ROOT}/tasks.c"
    "${FREERTOS_ROOT}/queue.c"
    "${FREERTOS_ROOT}/list.c"
    "${FREERTOS_ROOT}/event_groups.c"
    "${FREERTOS_ROOT}/timers.c"
    "${FREERTOS_ROOT}/stream_buffer.c"
    # umsg core + FreeRTOS port + generated message types
    "${UMSG_ROOT}/src/umsg.c"
    "${UMSG_ROOT}/src/port_freertos.c"
    ${GENERATED_MSG_SRCS}
)
set(VENDOR_INCS
    "${NANOPB_ROOT}"
    "${FREERTOS_ROOT}/include"
    "${UMSG_ROOT}/inc"
    "${GENERATED_MSG_INCS}"
)

# ── App group ────────────────────────────────────────────────────────────────
set(APP_DEFS "")
set(APP_INCS
    "${FIRMWARE_ROOT}/inc"
)

file(GLOB_RECURSE APP_SRCS
    "${FIRMWARE_ROOT}/src/*.c"
    "${FIRMWARE_ROOT}/src/*.s"
)

set(TARGETS "")
set(CFLAGS
    -Wall
    -Wextra
    -Wbad-function-cast
    -Wcast-align
    -Wcast-qual
    -Wfloat-equal
    -Wformat=2
    -Winline
    -Wnested-externs
    -Wpointer-arith
    -Wredundant-decls
    -Wshadow
    -Wstrict-prototypes
    -Wswitch
    -Wundef
    -Wunreachable-code
    -Wunused
    -Wwrite-strings
    -Wstrict-aliasing
)
list(JOIN CFLAGS " " CFLAGS_STRING)

if(HIL_TEST STREQUAL "True")

    file(GLOB_RECURSE HIL_SRCS
        "${TESTS_ROOT}/HIL/*.c"
        "${TESTS_ROOT}/HIL/*.s"
        "${TESTS_ROOT}/unity/*.c"
        "${TESTS_ROOT}/unity/*.s"
    )
    set(HIL_INCS
        "${TESTS_ROOT}/HIL"
        "${TESTS_ROOT}/unity"
    )
    list(APPEND APP_INCS "${HIL_INCS}")
    list(APPEND APP_SRCS "${HIL_SRCS}")
    list(APPEND APP_DEFS "UNITY_INCLUDE_CONFIG_H")

else()

    list(APPEND APP_SRCS "${FIRMWARE_ROOT}/main.c")

endif()

include("${TARGET_ROOT}/target.cmake")

message(STATUS "Building for board: ${BOARD_NAME}")
message(STATUS "Board defines: ${BOARD_DEFINES}")

# ── Apply shared groups to all registered targets ────────────────────────────
foreach(target IN LISTS TARGETS)
    target_sources(${target} PRIVATE
        ${VENDOR_SRCS}
        ${PLATFORM_SRCS}
        ${APP_SRCS}
    )
    target_compile_definitions(${target} PRIVATE
        ${PLATFORM_DEFS}
        ${BOARD_DEFINES}
        ${APP_DEFS}
    )
    target_include_directories(${target} SYSTEM PRIVATE
        ${VENDOR_INCS}
        ${PLATFORM_INCS}
    )
    target_include_directories(${target} PRIVATE
        ${APP_INCS}
        ${TARGET_INCS}
    )
    set_source_files_properties(${APP_SRCS}
        TARGET_DIRECTORY ${target}
        PROPERTIES COMPILE_FLAGS ${CFLAGS_STRING}
    )
endforeach()

set(SIZE_COMMANDS "")
foreach(target IN LISTS TARGETS)
    list(APPEND SIZE_COMMANDS
        COMMAND ${CMAKE_SIZE} -A $<TARGET_FILE:${target}>
    )
endforeach()

add_custom_target(size ALL
    DEPENDS ${TARGETS}
    ${SIZE_COMMANDS}
    COMMENT "Reporting size for all MCU cores: ${TARGETS}"
)
