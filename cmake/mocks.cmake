
if(NOT DEFINED cmock_SOURCE_DIR)
    message(FATAL_ERROR "CMock must be fetched before including mocks.cmake")
endif()

if(NOT DEFINED unity_SOURCE_DIR)
    message(FATAL_ERROR "Unity must be fetched before including mocks.cmake")
endif()

find_program(RUBY_EXECUTABLE ruby)
if(NOT RUBY_EXECUTABLE)
    message(FATAL_ERROR "Ruby not found! CMock requires Ruby to generate mocks. Install Ruby (ruby installer for windows) and ensure it's in the PATH.")
endif()

message(STATUS "Found Ruby: ${RUBY_EXECUTABLE}")
set(CMOCK_EXEC ${RUBY_EXECUTABLE} ${cmock_SOURCE_DIR}/lib/cmock.rb)
# Create mock base output directory
file(MAKE_DIRECTORY ${CMOCK_OUTPUT_BASE_DIR})

function(generate_mock mock_name header_dir header_file output_sub_dir)

    set(CMOCK_HEADER_PATH ${header_dir}/${header_file}.h)
    set(CMOCK_OUTPUT_PATH ${CMOCK_OUTPUT_BASE_DIR}) #/${output_sub_dir})
    set(CMOCK_OUTPUT_PATH_H ${CMOCK_OUTPUT_PATH}/mock_${header_file}.h)
    set(CMOCK_OUTPUT_PATH_C ${CMOCK_OUTPUT_PATH}/mock_${header_file}.c)

    add_custom_command(
        OUTPUT ${CMOCK_OUTPUT_PATH_C} ${CMOCK_OUTPUT_PATH_H}
        COMMAND ${CMOCK_EXEC} -o${CMOCK_CONFIG_FILE} --mock_path=${CMOCK_OUTPUT_PATH} ${CMOCK_HEADER_PATH}
        DEPENDS ${CMOCK_HEADER_PATH}
        COMMENT "Generating mock for ${header_file}.h"
        VERBATIM
    )

    add_custom_target(generate_${mock_name}_mock 
        DEPENDS ${CMOCK_OUTPUT_PATH_C} ${CMOCK_OUTPUT_PATH_H}
    )

    add_library(${mock_name} STATIC ${CMOCK_OUTPUT_PATH_C})
    target_include_directories(${mock_name} PUBLIC 
        ${CMOCK_OUTPUT_PATH}
        ${FJ_TESTS_DIR}/stubs
        ${unity_SOURCE_DIR}/src
        ${cmock_SOURCE_DIR}/src
    )
    target_link_libraries(${mock_name} PUBLIC unity)
    target_compile_definitions(${mock_name} PUBLIC UNIT_TEST)
    add_dependencies(${mock_name} generate_${mock_name}_mock)

endfunction()


generate_mock(hal_gpio ${STM32_HAL_INC_DIR} stm32h7xx_hal_gpio stm32)
generate_mock(hal_spi ${STM32_HAL_INC_DIR} stm32h7xx_hal_spi stm32)
generate_mock(hal_uart ${STM32_HAL_INC_DIR} stm32h7xx_hal_uart stm32)
generate_mock(hal_i2c ${STM32_HAL_INC_DIR} stm32h7xx_hal_i2c stm32)