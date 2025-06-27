cmake_minimum_required(VERSION 3.15)
project(IMU_Tests C)

include(CTest)
enable_testing()

include(FetchContent)
FetchContent_Declare(
    unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG        v2.5.2
)
FetchContent_MakeAvailable(unity)

# Add IMU API include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/../Common/Inc
)

set(TEST_SOURCES
    test_imu.c
    ./stubs/hal_stub.c
    ../Common/Src/sensors/imu/imu.c
    ../Common/Src/common.c
)

add_executable(test_imu ${TEST_SOURCES})
target_compile_definitions(test_imu PRIVATE UNIT_TEST)
target_include_directories(test_imu PRIVATE ${unity_SOURCE_DIR}/src)
target_sources(test_imu PRIVATE ${unity_SOURCE_DIR}/src/unity.c)

add_test(NAME IMU_Tests COMMAND test_imu)
