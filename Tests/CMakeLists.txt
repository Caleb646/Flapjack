cmake_minimum_required(VERSION 3.20)

project(FlapjackTests C)

include(CTest)
enable_testing()

cmake_path(GET CMAKE_CURRENT_SOURCE_DIR PARENT_PATH PRJ_ROOT)
set(TEST_ROOT "${PRJ_ROOT}/Tests")
set(UNITTEST_ROOT "${TEST_ROOT}/UnitTest")

set(FIRMWARE_ROOT "${PRJ_ROOT}/Firmware")

# Unity is vendored in Tests/unity rather than fetched: every test includes it as
# "unity/unity.h", which only resolves with Tests/ on the include path. Fetching
# it put the header at <build>/_deps/unity-src/src/unity.h instead, so no test
# could ever compile - and it made the suite need the network to configure.
set(UNITY_SRC "${TEST_ROOT}/unity/unity.c")
set(UNITY_INC "${TEST_ROOT}")

set(COMMON_SRC_DIR "${PRJ_ROOT}/Firmware")
set(COMMON_INC_DIR "${PRJ_ROOT}/Firmware")
set(TARGETS_INC_DIR "${PRJ_ROOT}/Firmware/target")

set(COMMON_DEFS UNIT_TEST BOARD_NUCLEO_H747ZI)

# MinGW defaults to -mms-bitfields (MSVC bitfield layout), under which
# CrsfChannelsPayload_t packs to 32 bytes instead of the 22 the ARM EABI target
# produces. The host build would then decode a completely different wire layout
# to the firmware, making any CRSF channel test meaningless. Force GNU layout so
# the host matches the target; test_crsf.c static-asserts the resulting size.
if(MINGW)
    add_compile_options(-mno-ms-bitfields)
endif()

add_library(drone_common_obj OBJECT
    ${COMMON_SRC_DIR}/core/core.c
    ${COMMON_SRC_DIR}/core/core_shared.c
    ${COMMON_SRC_DIR}/core/sync.c
    ${COMMON_SRC_DIR}/core/log/logger.c
    ${COMMON_SRC_DIR}/core/log/format.c
    ${COMMON_SRC_DIR}/common/vector.c
    ${COMMON_SRC_DIR}/common/queue.c
    ${COMMON_SRC_DIR}/common/umap.c
    ${COMMON_SRC_DIR}/common/ring_buff.c
    ${UNITTEST_ROOT}/stubs/hal_stub.c
)

add_library(drone_peripherals_obj OBJECT
    ${COMMON_SRC_DIR}/drivers/dma.c
    ${COMMON_SRC_DIR}/drivers/io/gpio.c
    ${COMMON_SRC_DIR}/drivers/timer.c
)

# crsf.c only. Its UART dependency (UartPort_Init / UartPort_Write, reached via
# Crsf_Init and the unused Crsf_Bind) is satisfied by small stubs in the test
# itself, because neither uart.c nor spi.c compiles against stubs/hal_stub.c -
# they need HAL macros (__HAL_UART_GET_FLAG, LL_SPI_*) the stub does not define.
# Nothing under test drives a real port, so pulling them in would only mean
# stubbing more HAL to reach the same place.
add_library(drone_rx_obj OBJECT
    ${COMMON_SRC_DIR}/drivers/rx/crsf.c
)

# gps.c + its vendored minmea parser. Same arrangement as drone_rx_obj: the UART
# dependency (UartPort_Init, reached via Gps_Init_) is stubbed in the test itself
# rather than pulling in uart.c, which does not build against hal_stub.c.
add_library(drone_gps_obj OBJECT
    ${COMMON_SRC_DIR}/drivers/gps/gps.c
    ${COMMON_SRC_DIR}/drivers/gps/parser/minmea.c
)

# align.c only. Pure integer rotation tables with no FreeRTOS, umsg or driver
# dependency, for the same reason filter.c is reachable here: a typo in the
# table is invisible on a bench, so the host is where it has to be pinned down.
add_library(drone_align_obj OBJECT
    ${COMMON_SRC_DIR}/common/align.c
)

# filter.c only. The altitude estimator is deliberately pure maths with no
# FreeRTOS, umsg or driver dependency, precisely so it can be exercised here -
# Nav_Update cannot be, so the filter is where the vertical maths has to be
# pinned down. It pulls in GetMicroseconds via the Madgwick half, which
# drone_common_obj already provides.
add_library(drone_filter_obj OBJECT
    ${COMMON_SRC_DIR}/common/filter.c
)

foreach(lib
    drone_common_obj
    drone_peripherals_obj
    drone_rx_obj
    drone_gps_obj
    drone_filter_obj
    drone_align_obj
)
    target_include_directories(${lib} PRIVATE
        ${COMMON_INC_DIR}
        ${COMMON_INC_DIR}/platform
        ${TARGETS_INC_DIR}
        ${UNITTEST_ROOT}/stubs
    )
    target_compile_definitions(${lib} PRIVATE ${COMMON_DEFS})
endforeach()

function(add_test_exe test_name)
    cmake_parse_arguments(TEST "" "" "OBJECTS" ${ARGN})

    add_executable(${test_name} ${UNITTEST_ROOT}/${test_name}.c)
    target_sources(${test_name} PRIVATE ${UNITY_SRC})
    target_include_directories(${test_name} PRIVATE
        ${UNITY_INC}
        ${COMMON_INC_DIR}
        ${COMMON_INC_DIR}/platform
        ${TARGETS_INC_DIR}
        ${UNITTEST_ROOT}/stubs
    )
    target_compile_definitions(${test_name} PRIVATE ${COMMON_DEFS})
    target_link_libraries(${test_name} PRIVATE drone_common_obj ${TEST_OBJECTS} m)

    add_test(NAME ${test_name} COMMAND ${test_name})
endfunction()

add_test_exe(test_crsf OBJECTS drone_common_obj drone_peripherals_obj drone_rx_obj)
add_test_exe(test_gps  OBJECTS drone_common_obj drone_peripherals_obj drone_gps_obj)
add_test_exe(test_altitude OBJECTS drone_common_obj drone_filter_obj)
add_test_exe(test_align OBJECTS drone_common_obj drone_align_obj)
