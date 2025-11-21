include(CTest)
enable_testing()

include(FetchContent)
FetchContent_Declare(unity GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git GIT_TAG v2.5.2)
FetchContent_MakeAvailable(unity)

FetchContent_Declare(cmock GIT_REPOSITORY https://github.com/ThrowTheSwitch/CMock.git GIT_TAG v2.5.3)
FetchContent_MakeAvailable(cmock)

include (mocks)

add_library(drone_common_obj OBJECT
    ${FJ_SRC_DIR}/core/core_shared.c
    ${FJ_SRC_DIR}/core/core.c
    # ${FJ_SRC_DIR}/core/sync.c
    # ${FJ_SRC_DIR}/core/log/format.c
    # ${FJ_SRC_DIR}/core/log/logger.c

    # ${FJ_SRC_DIR}/mem/vector.c
    # ${FJ_SRC_DIR}/mem/queue.c
    # ${FJ_SRC_DIR}/mem/umap.c
    # ${FJ_SRC_DIR}/mem/ring_buff.c

    ${FJ_TESTS_DIR}/stubs/hal_stub.c
)

# add_library(drone_base_peripherals_obj OBJECT
#     ${FJ_SRC_DIR}/peripheral/dma.c
#     ${FJ_SRC_DIR}/peripheral/gpio.c
#     ${FJ_SRC_DIR}/peripheral/timer.c
# )

# add_library(drone_bus_peripherals_obj OBJECT
#     ${FJ_SRC_DIR}/peripheral/bus/spi.c
#     ${FJ_SRC_DIR}/peripheral/bus/uart.c
#     ${FJ_SRC_DIR}/peripheral/bus/i2c.c
#     ${FJ_SRC_DIR}/peripheral/bus/bus.c
# )

# add_library(drone_device_obj OBJECT
#     ${FJ_SRC_DIR}/device/imu/imu.c
#     ${FJ_SRC_DIR}/device/mag/mag.c
#     ${FJ_SRC_DIR}/device/serial/serial.c
# )

# add_library(drone_movement_obj OBJECT
#     ${FJ_SRC_DIR}/device/motor/motor.c
#     ${FJ_SRC_DIR}/device/servo/servo.c
#     ${FJ_SRC_DIR}/mc/actuators.c
# )

# add_library(drone_mc_obj OBJECT
#     ${FJ_SRC_DIR}/fcstate.c
#     ${FJ_SRC_DIR}/mc/dshot.c
#     ${FJ_SRC_DIR}/mc/filter.c
#     ${FJ_SRC_DIR}/mc/pid.c
# )

foreach(lib drone_common_obj) #drone_base_peripherals_obj drone_movement_obj drone_bus_peripherals_obj drone_device_obj drone_mc_obj)
    target_include_directories( ${lib} PRIVATE 
                                ${FJ_INC_DIR} 
                                ${CMOCK_OUTPUT_BASE_DIR}
                                ${unity_SOURCE_DIR}/src
                                ${FJ_TESTS_DIR}/stubs
                                )
    target_compile_definitions(${lib} PRIVATE UNIT_TEST) # STM32H747xx CORE_CM7 USE_HAL_DRIVER)
endforeach()