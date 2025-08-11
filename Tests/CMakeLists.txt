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

# Find Python3 for running the test registration generator

# # Custom command to generate registered_tests.c from test files
# add_custom_command(
#     OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/registered_tests.c
#     COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/register_tests.py
#     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#     DEPENDS 
#         ${CMAKE_CURRENT_SOURCE_DIR}/register_tests.py
#     COMMENT "Auto-generating test registrations from test files..."
#     VERBATIM
# )

# # Create a custom target that depends on the generated file
# add_custom_target(register_tests 
#     DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/registered_tests.c
# )
find_package(Python3 COMPONENTS Interpreter REQUIRED)

# Generate the file during configure time if it doesn't exist
# if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/registered_tests.c")
message(STATUS "Generating initial registered_tests.c...")
execute_process(
    COMMAND ${Python3_EXECUTABLE} "register_tests.py"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE SCRIPT_RESULT
    OUTPUT_VARIABLE SCRIPT_OUTPUT
    ERROR_VARIABLE SCRIPT_ERROR
)
if(NOT SCRIPT_RESULT EQUAL 0)
    message(WARNING "Failed to generate registered_tests.c: ${SCRIPT_ERROR}")
    message(STATUS "Creating empty registered_tests.c as fallback...")
    file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/registered_tests.c" 
        "// Auto-generated fallback file\n"
        "#include \"test_runner.h\"\n"
        "void register_all_tests(void) {\n"
        "    // No tests registered - please run the Python script manually\n"
        "}\n"
    )
else()
    message(STATUS "Successfully generated registered_tests.c")
endif()
# endif()



# add_custom_target(regenerate_tests
#     COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/register_tests.py
#     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#     COMMENT "Manually regenerating test registrations..."
#     VERBATIM
# )

# Combined test sources including all test files and their dependencies
set(ALL_TEST_SOURCES
    test_runner.c
    test_imu.c
    test_filter.c
    test_queue.c
    test_actuators.c
    registered_tests.c
    ./stubs/hal_stub.c
    ../Common/Src/sensors/imu/imu.c
    ../Common/Src/mc/filter.c
    ../Common/Src/mem/queue.c
    ../Common/Src/common.c
    # ../Common/Src/log/format.c
)

# Build single test executable that includes all tests
add_executable(test_runner ${ALL_TEST_SOURCES})
target_compile_definitions(test_runner PRIVATE UNIT_TEST)
target_include_directories(test_runner PRIVATE ${unity_SOURCE_DIR}/src)
target_sources(test_runner PRIVATE ${unity_SOURCE_DIR}/src/unity.c)

# Make sure the test executable depends on the generated registrations
# add_dependencies(test_runner register_tests)
# add_dependencies(test_runner regenerate_tests)

# Add single test that runs all tests
add_test(NAME All_Tests COMMAND test_runner)
