cmake_minimum_required(VERSION 3.15)

project(DroneTests C)

include(CTest)
enable_testing()

include(FetchContent)
FetchContent_Declare(unity GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git GIT_TAG v2.5.2)
FetchContent_MakeAvailable(unity)

FetchContent_Declare(cmock GIT_REPOSITORY https://github.com/ThrowTheSwitch/CMock.git GIT_TAG v2.6.0)
FetchContent_MakeAvailable(cmock)

set(COMMON_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Common/Src")
set(COMMON_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Common/Inc")
set(MOCK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/mocks")
set(CMOCK_EXEC ruby ${cmock_SOURCE_DIR}/lib/cmock.rb)

# set(STM32_HAL_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/STM32H7xx_HAL_Driver/Inc")
# set(STM32_CMSIS_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/CMSIS/Device/ST/STM32H7xx/Include")
# set(STM32_CMSIS_CORE_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/CMSIS/Include")

add_library(drone_common_obj OBJECT
    ${COMMON_SRC_DIR}/core/core_shared.c
    ${COMMON_SRC_DIR}/core/core.c
    ${COMMON_SRC_DIR}/core/sync.c
    ${COMMON_SRC_DIR}/core/log/format.c
    ${COMMON_SRC_DIR}/core/log/logger.c

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
    ${COMMON_SRC_DIR}/fcstate.c
    ${COMMON_SRC_DIR}/mc/dshot.c
    ${COMMON_SRC_DIR}/mc/filter.c
    ${COMMON_SRC_DIR}/mc/pid.c
)

foreach(lib drone_common_obj drone_base_peripherals_obj drone_movement_obj drone_bus_peripherals_obj drone_device_obj drone_mc_obj)
    target_include_directories( ${lib} PRIVATE 
                                ${COMMON_INC_DIR} 
                                ${MOCK_DIR}
                                stubs
                                # ${STM32_HAL_INC_DIR} 
                                # ${STM32_CMSIS_INC_DIR}
                                # ${STM32_CMSIS_CORE_INC_DIR}
                                )
    target_compile_definitions(${lib} PRIVATE UNIT_TEST) # STM32H747xx CORE_CM7 USE_HAL_DRIVER)
endforeach()

# Function to generate mocks for HAL headers
function(add_mock_target mock_name header_file)
    set(MOCK_OUTPUT_DIR ${MOCK_DIR}/${mock_name})
    set(MOCK_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmock_config.yml)
    
    # Create mock output directory
    file(MAKE_DIRECTORY ${MOCK_OUTPUT_DIR})
    
    # Generate mock files
    add_custom_command(
        OUTPUT ${MOCK_OUTPUT_DIR}/mock_${header_file}.c ${MOCK_OUTPUT_DIR}/mock_${header_file}.h
        COMMAND ${CMOCK_EXEC} -o${MOCK_CONFIG_FILE} ${STM32_HAL_INC_DIR}/${header_file}.h
        COMMAND ${CMAKE_COMMAND} -E copy mock_${header_file}.c ${MOCK_OUTPUT_DIR}/
        COMMAND ${CMAKE_COMMAND} -E copy mock_${header_file}.h ${MOCK_OUTPUT_DIR}/
        COMMAND ${CMAKE_COMMAND} -E remove mock_${header_file}.c mock_${header_file}.h
        DEPENDS ${STM32_HAL_INC_DIR}/${header_file}.h ${MOCK_CONFIG_FILE}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating mock for ${header_file}.h"
    )
    
    add_custom_target(${mock_name}_mock
        DEPENDS ${MOCK_OUTPUT_DIR}/mock_${header_file}.c ${MOCK_OUTPUT_DIR}/mock_${header_file}.h
    )
endfunction()

# Generate mocks for HAL modules
add_mock_target(hal_spi stm32h7xx_hal_spi)
add_mock_target(hal_uart stm32h7xx_hal_uart)
add_mock_target(hal_i2c stm32h7xx_hal_i2c)
add_mock_target(hal_gpio stm32h7xx_hal_gpio)

function(add_test_exe test_name test_source)
    cmake_parse_arguments(TEST "" "" "OBJECTS;MOCKS" ${ARGN})
    
    add_executable(${test_name} ${test_source})
    target_sources(${test_name} PRIVATE ${unity_SOURCE_DIR}/src/unity.c)
    target_include_directories(${test_name} PRIVATE ${unity_SOURCE_DIR}/src ${COMMON_INC_DIR} stubs)
    target_compile_definitions(${test_name} PRIVATE UNIT_TEST)
    target_link_libraries(${test_name} PRIVATE drone_common_obj ${TEST_OBJECTS})
    add_test(NAME ${test_name} COMMAND ${test_name})
endfunction()

set(DRONE_ALL_OBJECTS
    drone_common_obj
    drone_base_peripherals_obj 
    drone_bus_peripherals_obj
    drone_device_obj
    drone_movement_obj
    drone_mc_obj
)

# Core tests
add_test_exe(test_common core/test_common.c)
add_test_exe(test_filter core/test_filter.c OBJECTS DRONE_ALL_OBJECTS)

# Memory tests
add_test_exe(test_queue mem/test_queue.c)
add_test_exe(test_vector mem/test_vector.c)
add_test_exe(test_umap mem/test_umap.c)

# Peripheral tests
add_test_exe(test_gpio peripheral/test_gpio.c OBJECTS DRONE_ALL_OBJECTS)
add_test_exe(test_spi peripheral/test_spi.c OBJECTS DRONE_ALL_OBJECTS)
add_test_exe(test_uart peripheral/test_uart.c OBJECTS DRONE_ALL_OBJECTS)

# Device tests
add_test_exe(test_imu device/test_imu.c OBJECTS DRONE_ALL_OBJECTS)
add_test_exe(test_mag device/test_mag.c OBJECTS DRONE_ALL_OBJECTS)
add_test_exe(test_actuators device/test_actuators.c OBJECTS DRONE_ALL_OBJECTS)