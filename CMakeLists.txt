cmake_minimum_required(VERSION 3.15)

project(FlapJack C)

set(CMAKE_C_STANDARD 11)

set(FJ_TESTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Tests")
set(FJ_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Common/Src")
set(FJ_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Common/Inc")

set(CMOCK_OUTPUT_BASE_DIR "${FJ_TESTS_DIR}/mocks")
set(CMOCK_CONFIG_FILE "${FJ_TESTS_DIR}/cmock_config.yml")

set(STM32_HAL_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc")
set(STM32_CMSIS_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include")
set(STM32_CMSIS_CORE_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include")

# Add my cmake modules to the path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(BUILD_TESTS)
    include (tests)
endif()