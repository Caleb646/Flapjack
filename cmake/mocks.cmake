
if(NOT DEFINED cmock_SOURCE_DIR)
    message(FATAL_ERROR "CMock must be fetched before including mocks.cmake")
endif()

if(NOT DEFINED unity_SOURCE_DIR)
    message(FATAL_ERROR "Unity must be fetched before including mocks.cmake")
endif()

find_program(RUBY_EXE ruby)
if(NOT RUBY_EXE)
    message(FATAL_ERROR "Ruby not found! CMock requires Ruby to generate mocks. Install Ruby (ruby installer for windows) and ensure it's in the PATH.")
endif()

message(STATUS "Found Ruby: ${RUBY_EXE}")
set(CMOCK_EXEC ${RUBY_EXE} ${cmock_SOURCE_DIR}/lib/cmock.rb)
file(MAKE_DIRECTORY ${CMOCK_OUTPUT_BASE_DIR})

function(generate_mock mock_name header_dir header_file)

    set(TO_BE_MOCKED_HEADER_PATH ${header_dir}/${header_file}.h)
    set(MOCKED_OUTPUT_PATH ${CMOCK_OUTPUT_BASE_DIR})
    set(MOCKED_OUTPUT_PATH_H ${MOCKED_OUTPUT_PATH}/mock_${header_file}.h)
    set(MOCKED_OUTPUT_PATH_C ${MOCKED_OUTPUT_PATH}/mock_${header_file}.c)

    add_custom_command(
        OUTPUT ${MOCKED_OUTPUT_PATH_C} ${MOCKED_OUTPUT_PATH_H}
        COMMAND ${CMOCK_EXEC} -o${CMOCK_CONFIG_FILE} --mock_path=${MOCKED_OUTPUT_PATH} ${TO_BE_MOCKED_HEADER_PATH}
        DEPENDS ${TO_BE_MOCKED_HEADER_PATH}
        COMMENT "Generating mock for ${header_file}.h"
        VERBATIM
    )

    set_property(GLOBAL APPEND PROPERTY MOCKED_SOURCE_FILES ${MOCKED_OUTPUT_PATH_C})
    set_property(GLOBAL APPEND PROPERTY MOCKED_HEADER_FILES ${MOCKED_OUTPUT_PATH_H})
    set_property(GLOBAL APPEND PROPERTY MOCKED_HEADER_FILE_INCS ${MOCKED_OUTPUT_PATH})

    # cmock has no way of dealing with subdirectories. 
    # So #include device/imu/imu.h becomes #include imu.h in mock_imu.h .
    # To deal with this, include the full path of the original header file when building.
    string(FIND "${header_dir}" "${FJ_INC_DIR}" is_fj_header)
    if(NOT is_fj_header EQUAL -1)
        set_property(GLOBAL APPEND PROPERTY MOCKED_HEADER_FILE_INCS ${header_dir})
    endif()

endfunction()


generate_mock(hal_gpio ${STM32_HAL_INC_DIR} stm32h7xx_hal_gpio)
generate_mock(hal_spi ${STM32_HAL_INC_DIR} stm32h7xx_hal_spi)
generate_mock(hal_uart ${STM32_HAL_INC_DIR} stm32h7xx_hal_uart)
generate_mock(hal_i2c ${STM32_HAL_INC_DIR} stm32h7xx_hal_i2c)
generate_mock(hal_hsem ${STM32_HAL_INC_DIR} stm32h7xx_hal_hsem)
generate_mock(hal_dma ${STM32_HAL_INC_DIR} stm32h7xx_hal_dma)
generate_mock(hal_dma_ex ${STM32_HAL_INC_DIR} stm32h7xx_hal_dma_ex)


generate_mock(fj_imu ${FJ_INC_DIR}/device/imu imu)
generate_mock(fj_mag ${FJ_INC_DIR}/device/mag mag)
generate_mock(fj_filter ${FJ_INC_DIR}/mc filter)