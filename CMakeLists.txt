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
set(MCU_ROOT "${FIRMWARE_ROOT}/mcu")
set(TARGET_ROOT "${FIRMWARE_ROOT}/target")

set(FIRMWARE_DEFINES "")

set(FIRMWARE_INCS
    "${FIRMWARE_ROOT}/inc"
)

file(GLOB_RECURSE FIRMWARE_SRCS
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
    list(APPEND FIRMWARE_INCS "${HIL_INCS}")
    list(APPEND FIRMWARE_SRCS "${HIL_SRCS}")
    list(APPEND FIRMWARE_DEFINES "UNITY_INCLUDE_CONFIG_H")

else()

    list(APPEND FIRMWARE_SRCS "${FIRMWARE_ROOT}/main.c")

endif()

include("${TARGET_ROOT}/target.cmake")

message(STATUS "Building for board: ${BOARD_NAME}")
message(STATUS "Board defines: ${BOARD_DEFINES}")

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