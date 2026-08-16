cmake_minimum_required(VERSION 3.20)

project(FlapjackTests C)

include(CTest)
enable_testing()

cmake_path(GET CMAKE_CURRENT_SOURCE_DIR PARENT_PATH PRJ_ROOT)
set(TEST_ROOT "${PRJ_ROOT}/Tests")
set(HIL_ROOT "${TEST_ROOT}/HIL")
set(UNITTEST_ROOT "${TEST_ROOT}/UnitTest")

set(FIRMWARE_ROOT "${PRJ_ROOT}/Firmware")

include(FetchContent)
FetchContent_Declare(unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG        v2.5.2
)
FetchContent_MakeAvailable(unity)

set(UNITY_SRC "${unity_SOURCE_DIR}/src/unity.c")
set(UNITY_INC "${unity_SOURCE_DIR}/src")

set(COMMON_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Firmware")
set(COMMON_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Firmware")
set(TARGETS_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Firmware/target")

set(COMMON_DEFS UNIT_TEST BOARD_NUCLEO_H747ZI)

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

add_library(drone_bus_obj OBJECT
    ${COMMON_SRC_DIR}/drivers/bus/spi.c
    ${COMMON_SRC_DIR}/drivers/serial/uart.c
)

add_library(drone_device_obj OBJECT
    ${COMMON_SRC_DIR}/drivers/imu/bmi323.c
    ${COMMON_SRC_DIR}/drivers/imu/sim.c
)

add_library(drone_mc_obj OBJECT
    ${COMMON_SRC_DIR}/drivers/dshot/dshot.c
    ${COMMON_SRC_DIR}/common/filter.c
    ${COMMON_SRC_DIR}/common/pid.c
)

foreach(lib
    drone_common_obj
    drone_peripherals_obj
    drone_bus_obj
    drone_device_obj
    drone_mc_obj
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

add_test_exe(test_queue   OBJECTS drone_common_obj)
add_test_exe(test_vector  OBJECTS drone_common_obj)
add_test_exe(test_umap    OBJECTS drone_common_obj)
add_test_exe(test_gpio    OBJECTS drone_common_obj drone_peripherals_obj)
add_test_exe(test_spi     OBJECTS drone_common_obj drone_peripherals_obj drone_bus_obj)
add_test_exe(test_uart    OBJECTS drone_common_obj drone_peripherals_obj drone_bus_obj)
add_test_exe(test_imu     OBJECTS drone_common_obj drone_peripherals_obj drone_bus_obj drone_device_obj drone_mc_obj)
add_test_exe(test_filter  OBJECTS drone_common_obj drone_peripherals_obj drone_bus_obj drone_device_obj drone_mc_obj)
add_test_exe(test_dshot   OBJECTS drone_common_obj drone_peripherals_obj drone_mc_obj)
