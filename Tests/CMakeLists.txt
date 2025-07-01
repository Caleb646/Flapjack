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

# Combined test sources including all test files and their dependencies
set(ALL_TEST_SOURCES
    test_runner.c
    test_imu.c
    test_filter.c
    ./stubs/hal_stub.c
    ../Common/Src/sensors/imu/imu.c
    ../Common/Src/motion_control/filter.c
    ../Common/Src/common.c
)

# Build single test executable that includes all tests
add_executable(test_runner ${ALL_TEST_SOURCES})
target_compile_definitions(test_runner PRIVATE UNIT_TEST)
target_include_directories(test_runner PRIVATE ${unity_SOURCE_DIR}/src)
target_sources(test_runner PRIVATE ${unity_SOURCE_DIR}/src/unity.c)

# Add single test that runs all tests
add_test(NAME All_Tests COMMAND test_runner)
