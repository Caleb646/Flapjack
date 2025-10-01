cmake_minimum_required(VERSION 3.15)

set (CMAKE_SYSTEM_PROCESSOR "arm" CACHE STRING "")
set (CMAKE_SYSTEM_NAME "Generic" CACHE STRING "")
# Skip link step during toolchain validation.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(TOOLCHAIN_PREFIX   "arm-none-eabi-")
set(CMAKE_C_COMPILER   "${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PREFIX}g++")
set(CMAKE_AR           "${TOOLCHAIN_PREFIX}ar")
set(CMAKE_LINKER       "{TOOLCHAIN_PREFIX}ld")
set(CMAKE_OBJCOPY      "${TOOLCHAIN_PREFIX}objcopy")
set(CMAKE_RANLIB       "${TOOLCHAIN_PREFIX}ranlib")
set(CMAKE_SIZE         "${TOOLCHAIN_PREFIX}size")
set(CMAKE_STRIP        "${TOOLCHAIN_PREFIX}ld")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

project(DroneTests C)

include(CTest)
enable_testing()

include(FetchContent)
FetchContent_Declare(unity GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git GIT_TAG v2.5.2)
FetchContent_MakeAvailable(unity)

set(COMMON_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Common/Src")
set(COMMON_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Common/Inc")
set(STM32_HAL_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/STM32H7xx_HAL_Driver/Inc")
set(STM32_CMSIS_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/CMSIS/Device/ST/STM32H7xx/Include")
set(STM32_CMSIS_CORE_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/CMSIS/Include")

add_library(drone_common_obj OBJECT
    ${COMMON_SRC_DIR}/common.c
    stubs/hal_stub.c
)

add_library(drone_mem_obj OBJECT
    ${COMMON_SRC_DIR}/mem/vector.c
    ${COMMON_SRC_DIR}/mem/queue.c
    ${COMMON_SRC_DIR}/mem/umap.c
    ${COMMON_SRC_DIR}/mem/ring_buff.c
)

add_library(drone_base_peripherals OBJECT
    ${COMMON_SRC_DIR}/peripheral/dma.c
    #${COMMON_SRC_DIR}/peripheral/gpio.c
    #${COMMON_SRC_DIR}/peripheral/timer.c
)

# add_library(drone_device_obj OBJECT
#     ${COMMON_SRC_DIR}/device/imu/imu.c
#     ${COMMON_SRC_DIR}/device/mag/mag.c
# )

# add_library(drone_mc_obj OBJECT
#     ${COMMON_SRC_DIR}/mc/filter.c
#     ${COMMON_SRC_DIR}/mc/actuators.c
# )

# foreach(lib drone_common_obj drone_mem_obj drone_device_obj drone_mc_obj)
foreach(lib drone_common_obj drone_mem_obj drone_base_peripherals)
    target_include_directories( ${lib} PRIVATE 
                                ${COMMON_INC_DIR} 
                                stubs
                                ${STM32_HAL_INC_DIR} 
                                ${STM32_CMSIS_INC_DIR}
                                ${STM32_CMSIS_CORE_INC_DIR}
                                )
    target_compile_definitions(${lib} PRIVATE UNIT_TEST STM32H747xx CORE_CM7 USE_HAL_DRIVER)
endforeach()

function(add_test_exe test_name)
    cmake_parse_arguments(TEST "" "" "OBJECTS" ${ARGN})
    
    add_executable(${test_name} ${test_name}.c)
    target_sources(${test_name} PRIVATE ${unity_SOURCE_DIR}/src/unity.c)
    target_include_directories( ${test_name} PRIVATE 
                                ${unity_SOURCE_DIR}/src 
                                ${COMMON_INC_DIR} 
                                stubs 
                                ${STM32_HAL_INC_DIR} 
                                ${STM32_CMSIS_INC_DIR}
                                ${STM32_CMSIS_CORE_INC_DIR}
                                )
    target_compile_definitions(${test_name} PRIVATE UNIT_TEST STM32H747xx CORE_CM7 USE_HAL_DRIVER __GNUC__)
    target_link_libraries(${test_name} PRIVATE drone_common_obj ${TEST_OBJECTS})
    
    add_test(NAME ${test_name} COMMAND ${test_name})
endfunction()

add_test_exe(test_queue OBJECTS drone_common_obj drone_mem_obj)

# add_test_exe(test_queue OBJECTS drone_mem_obj drone_device_obj)
# add_test_exe(test_imu OBJECTS drone_device_obj drone_mem_obj)
# add_test_exe(test_filter OBJECTS drone_mc_obj drone_mem_obj)
# add_test_exe(test_actuators OBJECTS drone_mc_obj drone_mem_obj)