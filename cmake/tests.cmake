include(CTest)
enable_testing()

include(FetchContent)
FetchContent_Declare(unity GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git GIT_TAG v2.5.2)
FetchContent_MakeAvailable(unity)

FetchContent_Declare(cmock GIT_REPOSITORY https://github.com/ThrowTheSwitch/CMock.git GIT_TAG v2.5.3)
FetchContent_MakeAvailable(cmock)

add_library(cmock_lib STATIC ${cmock_SOURCE_DIR}/src/cmock.c)
target_include_directories(cmock_lib PUBLIC ${cmock_SOURCE_DIR}/src ${unity_SOURCE_DIR}/src)
target_link_libraries(cmock_lib PUBLIC unity)

include (mocks)

get_property(ALL_MOCKED_SOURCES GLOBAL PROPERTY MOCKED_SOURCE_FILES)
get_property(ALL_MOCKED_HEADERS GLOBAL PROPERTY MOCKED_HEADER_FILES)
get_property(ALL_MOCKED_HEADER_FILE_INCS GLOBAL PROPERTY MOCKED_HEADER_FILE_INCS)

add_custom_target(generate_all_mocks 
    DEPENDS ${ALL_MOCKED_SOURCES} ${ALL_MOCKED_HEADERS}
)

# add_library(core_obj STATIC
#     ${FJ_SRC_DIR}/core/core_shared.c
#     ${FJ_SRC_DIR}/core/core.c
#     ${FJ_SRC_DIR}/core/sync.c
#     ${FJ_SRC_DIR}/core/log/format.c
#     ${FJ_SRC_DIR}/core/log/logger.c
#     ${FJ_SRC_DIR}/mem/vector.c
#     ${FJ_SRC_DIR}/mem/queue.c
#     ${FJ_SRC_DIR}/mem/umap.c
#     ${FJ_SRC_DIR}/mem/ring_buff.c
#     ${FJ_TESTS_DIR}/stubs/hal_stub.c

#     ${ALL_MOCKED_SOURCES}
# )

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

# add_library(mc_obj OBJECT
#     # ${FJ_SRC_DIR}/fcstate.c
#     # ${FJ_SRC_DIR}/mc/dshot.c
#     ${FJ_SRC_DIR}/mc/filter.c
#     # ${FJ_SRC_DIR}/mc/pid.c
# )

set(TEST_INCS  
    ${FJ_INC_DIR} 
    ${ALL_MOCKED_HEADER_FILE_INCS}
    ${unity_SOURCE_DIR}/src
    ${cmock_SOURCE_DIR}/src
    ${FJ_TESTS_DIR}/stubs

    ${STM32_HAL_INC_DIR}
    ${STM32_CMSIS_INC_DIR}
    ${STM32_CMSIS_CORE_INC_DIR}
)

set(TEST_DEFS 
    UNIT_TEST 
    STM32H747xx 
    CORE_CM7 
    USE_HAL_DRIVER
)

function(add_test_lib lib_name)

    cmake_parse_arguments(LIB "" "" "SOURCES;OBJECTS;EXCLUDE_MOCKS" ${ARGN})
    # Filter out excluded mocks if specified
    set(FILTERED_MOCK_SOURCES ${ALL_MOCKED_SOURCES})
    if(LIB_EXCLUDE_MOCKS)

        foreach(exclude_mock ${LIB_EXCLUDE_MOCKS})
            list(FILTER FILTERED_MOCK_SOURCES EXCLUDE REGEX "mock_${exclude_mock}")
        endforeach()

        set(DISPLAY_MOCKS ${FILTERED_MOCK_SOURCES})
        list(TRANSFORM DISPLAY_MOCKS REPLACE ".*/([^/]+)$" "\\1")
        message(STATUS "Filtered mocks for ${lib_name}: ${DISPLAY_MOCKS}")

    endif()
    
    add_library(lib_${lib_name} STATIC     
        ${LIB_SOURCES}
        ${FILTERED_MOCK_SOURCES}
        ${FJ_TESTS_DIR}/stubs/hal_stub.c
    )
    
    target_include_directories(lib_${lib_name} PRIVATE ${TEST_INCS})
    target_compile_definitions(lib_${lib_name} PRIVATE ${TEST_DEFS})

    set(PREFIXED_OBJECTS ${LIB_OBJECTS})
    list(TRANSFORM PREFIXED_OBJECTS PREPEND "lib_")
    target_link_libraries(lib_${lib_name} PRIVATE ${PREFIXED_OBJECTS} unity cmock_lib)

    add_dependencies(lib_${lib_name} generate_all_mocks)

endfunction()


function(add_test_exe test_name test_source)

    add_executable(test_${test_name} ${test_source})
    target_link_libraries(test_${test_name} PRIVATE lib_${test_name} unity cmock_lib)
    target_include_directories(test_${test_name} PRIVATE ${TEST_INCS})
    target_compile_definitions(test_${test_name} PRIVATE ${TEST_DEFS})
    add_test(NAME test_${test_name} COMMAND test_${test_name})

endfunction()


add_test_lib(core
    SOURCES 
        ${FJ_SRC_DIR}/core/core_shared.c
        ${FJ_SRC_DIR}/core/core.c
        ${FJ_SRC_DIR}/core/sync.c
        ${FJ_SRC_DIR}/core/log/format.c
        ${FJ_SRC_DIR}/core/log/logger.c
        ${FJ_SRC_DIR}/mem/vector.c
        ${FJ_SRC_DIR}/mem/queue.c
        ${FJ_SRC_DIR}/mem/umap.c
        ${FJ_SRC_DIR}/mem/ring_buff.c
)
add_test_exe(core ${FJ_TESTS_DIR}/src/core/test_core.c)


add_test_lib(filter
    SOURCES 
        ${FJ_SRC_DIR}/mc/filter.c
    OBJECTS
        core
    EXCLUDE_MOCKS
        filter
)
add_test_exe(filter ${FJ_TESTS_DIR}/src/mc/test_filter.c)

