cmake_minimum_required(VERSION 3.15)

project(DroneTests C)

include(CTest)
enable_testing()

include(FetchContent)
FetchContent_Declare(unity GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git GIT_TAG v2.5.2)
FetchContent_MakeAvailable(unity)

set(COMMON_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Common/Src")
set(COMMON_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Common/Inc")
# set(STM32_HAL_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/STM32H7xx_HAL_Driver/Inc")
# set(STM32_CMSIS_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/CMSIS/Device/ST/STM32H7xx/Include")
# set(STM32_CMSIS_CORE_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/CMSIS/Include")

add_library(drone_common_obj OBJECT
    ${COMMON_SRC_DIR}/common.c
    ${COMMON_SRC_DIR}/sync.c
    ${COMMON_SRC_DIR}/log/format.c
    ${COMMON_SRC_DIR}/log/logger.c

    ${COMMON_SRC_DIR}/mem/vector.c
    ${COMMON_SRC_DIR}/mem/queue.c
    ${COMMON_SRC_DIR}/mem/umap.c
    ${COMMON_SRC_DIR}/mem/ring_buff.c

    stubs/hal_stub.c
)

add_library(drone_base_peripherals_obj OBJECT
    ${COMMON_SRC_DIR}/peripheral/dma.c
    ${COMMON_SRC_DIR}/peripheral/gpio.c
    ${COMMON_SRC_DIR}/peripheral/timer.c
)

add_library(drone_bus_peripherals_obj OBJECT
    ${COMMON_SRC_DIR}/peripheral/bus/spi.c
    ${COMMON_SRC_DIR}/peripheral/bus/uart.c
    ${COMMON_SRC_DIR}/peripheral/bus/i2c.c
    ${COMMON_SRC_DIR}/peripheral/bus/bus.c
)

add_library(drone_device_obj OBJECT
    ${COMMON_SRC_DIR}/device/imu/imu.c
    ${COMMON_SRC_DIR}/device/mag/mag.c
    ${COMMON_SRC_DIR}/device/serial/serial.c
)

add_library(drone_movement_obj OBJECT
    ${COMMON_SRC_DIR}/device/motor/motor.c
    ${COMMON_SRC_DIR}/device/servo/servo.c
    ${COMMON_SRC_DIR}/mc/actuators.c
)


add_library(drone_mc_obj OBJECT
    ${COMMON_SRC_DIR}/mc/dshot.c
    ${COMMON_SRC_DIR}/mc/fcstate.c
    ${COMMON_SRC_DIR}/mc/filter.c
    ${COMMON_SRC_DIR}/mc/pid.c
)

foreach(lib drone_common_obj drone_base_peripherals_obj drone_movement_obj drone_bus_peripherals_obj drone_device_obj drone_mc_obj)
    target_include_directories( ${lib} PRIVATE 
                                ${COMMON_INC_DIR} 
                                stubs
                                # ${STM32_HAL_INC_DIR} 
                                # ${STM32_CMSIS_INC_DIR}
                                # ${STM32_CMSIS_CORE_INC_DIR}
                                )
    target_compile_definitions(${lib} PRIVATE UNIT_TEST) # STM32H747xx CORE_CM7 USE_HAL_DRIVER)
endforeach()

function(add_test_exe test_name)
    cmake_parse_arguments(TEST "" "" "OBJECTS" ${ARGN})
    
    add_executable(${test_name} ${test_name}.c)
    target_sources(${test_name} PRIVATE ${unity_SOURCE_DIR}/src/unity.c)
    target_include_directories( ${test_name} PRIVATE 
                                ${unity_SOURCE_DIR}/src 
                                ${COMMON_INC_DIR} 
                                stubs 
                                # ${STM32_HAL_INC_DIR} 
                                # ${STM32_CMSIS_INC_DIR}
                                # ${STM32_CMSIS_CORE_INC_DIR}
                                )
    target_compile_definitions(${test_name} PRIVATE UNIT_TEST) # STM32H747xx CORE_CM7 USE_HAL_DRIVER __GNUC__)
    target_link_libraries(${test_name} PRIVATE drone_common_obj ${TEST_OBJECTS})
    
    add_test(NAME ${test_name} COMMAND ${test_name})
endfunction()

add_test_exe(test_queue OBJECTS drone_common_obj)
add_test_exe(test_vector OBJECTS drone_common_obj)

add_test_exe(test_spi OBJECTS drone_common_obj drone_base_peripherals_obj drone_bus_peripherals_obj)
add_test_exe(test_uart OBJECTS drone_common_obj drone_base_peripherals_obj drone_bus_peripherals_obj)

add_test_exe(test_imu OBJECTS drone_common_obj drone_mc_obj drone_device_obj drone_base_peripherals_obj drone_bus_peripherals_obj)

# add_test_exe(test_queue OBJECTS drone_mem_obj drone_device_obj)

# add_test_exe(test_filter OBJECTS drone_mc_obj drone_mem_obj)
# add_test_exe(test_actuators OBJECTS drone_mc_obj drone_mem_obj)