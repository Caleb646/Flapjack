cmake_minimum_required(VERSION 3.16)

set(FJ_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(STM32_TARGET_TRIPLET arm-none-eabi)
set(CMAKE_TOOLCHAIN_FILE ${FJ_ROOT_DIR}/cmake/gcc_tc.cmake)

project(FlapJack C ASM)

set(CMSIS_ROOT_DIR "${FJ_ROOT_DIR}/Vendor/STM32CubeH7/Drivers/CMSIS")
set(CMSIS_INC_DIR "${CMSIS_ROOT_DIR}/Include")
set(CMSIS_DEVICE_INC_DIR "${CMSIS_ROOT_DIR}/Device/ST/STM32H7xx/Include")
set(CMSIS_DEVICE_SRC_DIR "${CMSIS_ROOT_DIR}/Device/ST/STM32H7xx/Source/Templates")

# set(STARTUP_FILE "${CMSIS_DEVICE_SRC_DIR}/gcc/startup_stm32h747xihx.s")
# set(BOOT_FILE "${CMSIS_DEVICE_SRC_DIR}/system_stm32h7xx_dualcore_boot_cm4_cm7.c")

set(CMSIS_INCS
    ${CMSIS_INC_DIR}
    ${CMSIS_DEVICE_INC_DIR}
)

set(CMSIS_SRCS
    # ${STARTUP_FILE}
    # ${BOOT_FILE}
)

set(STM32_HAL_ROOT_DIR "${FJ_ROOT_DIR}/Vendor/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver")
set(STM32_HAL_INC_DIR "${STM32_HAL_ROOT_DIR}/Inc")
set(STM32_HAL_SRC_DIR "${STM32_HAL_ROOT_DIR}/Src")
# TODO specific to cm7 only
set(STM32_HAL_INCS 
    "${STM32_HAL_INC_DIR}" 
    # "${FJ_ROOT_DIR}/Firmware/Inc/platform/stm32/stm32h7xx"
) 
# file(GLOB_RECURSE STM32_HAL_SRCS "${STM32_HAL_SRC_DIR}/*.c")
set(STM32_HAL_SRCS
    ${STM32_HAL_SRC_DIR}/stm32h7xx_hal.c
    ${STM32_HAL_SRC_DIR}/stm32h7xx_hal_cortex.c
    ${STM32_HAL_SRC_DIR}/stm32h7xx_hal_gpio.c
    ${STM32_HAL_SRC_DIR}/stm32h7xx_hal_rcc.c
    ${STM32_HAL_SRC_DIR}/stm32h7xx_hal_rcc_ex.c
    ${STM32_HAL_SRC_DIR}/stm32h7xx_hal_pwr.c
    ${STM32_HAL_SRC_DIR}/stm32h7xx_hal_pwr_ex.c
    ${STM32_HAL_SRC_DIR}/stm32h7xx_hal_hsem.c
    ${STM32_HAL_SRC_DIR}/stm32h7xx_hal_uart.c
    ${STM32_HAL_SRC_DIR}/stm32h7xx_hal_uart_ex.c
    ${STM32_HAL_SRC_DIR}/stm32h7xx_hal_spi.c
    ${STM32_HAL_SRC_DIR}/stm32h7xx_hal_spi_ex.c
    ${STM32_HAL_SRC_DIR}/stm32h7xx_hal_i2c.c
    ${STM32_HAL_SRC_DIR}/stm32h7xx_hal_i2c_ex.c
    ${STM32_HAL_SRC_DIR}/stm32h7xx_hal_tim.c
    ${STM32_HAL_SRC_DIR}/stm32h7xx_hal_tim_ex.c
    ${STM32_HAL_SRC_DIR}/stm32h7xx_hal_dma.c
    ${STM32_HAL_SRC_DIR}/stm32h7xx_hal_dma_ex.c
    ${STM32_HAL_SRC_DIR}/stm32h7xx_hal_flash.c
    ${STM32_HAL_SRC_DIR}/stm32h7xx_hal_flash_ex.c
    # LL HAL drivers
    ${STM32_HAL_SRC_DIR}/stm32h7xx_ll_spi.c
)

set(FJ_SRC_DIR "${FJ_ROOT_DIR}/Firmware/Src")
set(FJ_INC_DIR "${FJ_ROOT_DIR}/Firmware/Inc")
# file(GLOB_RECURSE FJ_FIRMWARE_SRCS "${FJ_ROOT_DIR}/Firmware/Src/*.c")

set(FJ_FIRMWARE_INCS
    ${FJ_INC_DIR}
)
set(FJ_FIRMWARE_SRCS 
    ${FJ_SRC_DIR}/main.c
    ${FJ_SRC_DIR}/init.c
    ${FJ_SRC_DIR}/stl/ring.c
    ${FJ_SRC_DIR}/drivers/dma.c
    ${FJ_SRC_DIR}/drivers/motor.c
    ${FJ_SRC_DIR}/drivers/tim.c
    ${FJ_SRC_DIR}/drivers/core/log.c
    ${FJ_SRC_DIR}/drivers/core/lock.c
    ${FJ_SRC_DIR}/drivers/core/sys.c
    ${FJ_SRC_DIR}/drivers/bus/spi.c
    ${FJ_SRC_DIR}/drivers/io/gpio.c
    ${FJ_SRC_DIR}/drivers/serial/serial.c

    ${FJ_SRC_DIR}/drivers/sensors/sensor.c
    # TODO make sensor drivers configurable
    ${FJ_SRC_DIR}/drivers/sensors/imu/bmi323.c
    ${FJ_SRC_DIR}/drivers/sensors/mag/mmc5983.c
)

set(FJ_PLATFORM_INCS
    ${FJ_INC_DIR}/platform/stm32/stm32h7xx
)
set(FJ_PLATFORM_SRCS
# TODO make platform configurable
    ${FJ_SRC_DIR}/platform/stm32/dma.c
    ${FJ_SRC_DIR}/platform/stm32/tim.c
    ${FJ_SRC_DIR}/platform/stm32/gpio.c
    ${FJ_SRC_DIR}/platform/stm32/uart.c
    ${FJ_SRC_DIR}/platform/stm32/lock.c
    ${FJ_SRC_DIR}/platform/stm32/spi.c
    ${FJ_SRC_DIR}/platform/stm32/sys.c
    ${FJ_SRC_DIR}/platform/stm32/syscalls.c
    ${FJ_SRC_DIR}/platform/stm32/stm32h7xx/startup_stm32h747xihx.s
    ${FJ_SRC_DIR}/platform/stm32/stm32h7xx/system_stm32h7xx_dualcore_boot_cm4_cm7.c
)

set(FJ_TARG_INCS
    ${FJ_INC_DIR}
)
set(FJ_TARG_SRCS
# TODO make target configurable
    ${FJ_SRC_DIR}/targets/stm32h747i_disco.c
)


set(FJ_SHARED_CFLAGS
    # -std=c11
    # -pedantic
    # -pedantic-errors
    # -Werror
    -Wall
    -Wextra
    # -Waggregate-return
    -Wbad-function-cast
    -Wcast-align
    -Wcast-qual
    -Wfloat-equal
    -Wformat=2
    -Winline
    # -Wmissing-declarations
    # -Wmissing-prototypes
    -Wnested-externs
    -Wpointer-arith
    -Wredundant-decls
    -Wshadow
    -Wstrict-prototypes
    -Wswitch
    -Wundef
    -Wunreachable-code
    -Wunused
    -Wwrite-strings
    -Wstrict-aliasing
)

set(FJ_SHARED_DEFINES
    ${BOARD_CONFIG}
    USE_HAL_DRIVER
    USE_FULL_LL_DRIVER
    STM32H747xx
    # TODO needs to be configurable. This can brick the board if wrong.
    USE_PWR_DIRECT_SMPS_SUPPLY
    # USE_PWR_LDO_SUPPLY
)

add_executable(m7core 
    ${CMSIS_SRCS}
    ${STM32_HAL_SRCS}
    ${FJ_FIRMWARE_SRCS}
    ${FJ_PLATFORM_SRCS}
    ${FJ_TARG_SRCS}
)

add_custom_command(TARGET m7core POST_BUILD
    COMMAND ${CMAKE_SIZE} -A $<TARGET_FILE:m7core>
    COMMENT "Size information for m7core"
)

list(JOIN FJ_SHARED_CFLAGS " " FJ_SHARED_CFLAGS_STRING)
set_source_files_properties(${FJ_FIRMWARE_SRCS} PROPERTIES COMPILE_FLAGS ${FJ_SHARED_CFLAGS_STRING})
set_source_files_properties(${FJ_PLATFORM_SRCS} PROPERTIES COMPILE_FLAGS ${FJ_SHARED_CFLAGS_STRING})
set_source_files_properties(${FJ_TARG_SRCS} PROPERTIES COMPILE_FLAGS ${FJ_SHARED_CFLAGS_STRING})

target_compile_options(m7core PRIVATE 
    # use C11 standard with GNU extensions
    -std=gnu11
    # syscalls are simple stubs that return an error
    --specs=nosys.specs
    # use newlib nano for smaller footprint
    --specs=nano.specs
    -mcpu=cortex-m7
    -mthumb 
    -mfpu=fpv5-sp-d16 
    -mfloat-abi=hard 
    -fdata-sections 
    -ffunction-sections
)
target_link_options(m7core PRIVATE 
    -mcpu=cortex-m7 
    -mfpu=fpv5-sp-d16 
    -mfloat-abi=hard
    -fdata-sections 
    -ffunction-sections
    -static
    -mthumb
    --specs=nosys.specs
    --specs=nano.specs
    -T${CMAKE_CURRENT_SOURCE_DIR}/Scripts/cm7.ld
    -Wl,--gc-sections
    -Wl,--start-group -lc -lm -Wl,--end-group
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/m7core.map
)
target_compile_definitions(m7core PRIVATE ${FJ_SHARED_DEFINES} 
    -DCORE_CM7
    ${FJ_SHARED_DEFINES}
)
# use SYSTEM for cmsis and hal includes to suppress warnings from CMSIS and HAL
target_include_directories(m7core SYSTEM PRIVATE
    ${CMSIS_INCS}
    ${STM32_HAL_INCS}
)
target_include_directories(m7core PRIVATE
    ${FJ_FIRMWARE_INCS}
    ${FJ_PLATFORM_INCS}
    ${FJ_TARG_INCS}
)


# if(NOT BUILD_TESTS)

#     set(STM32_CMAKE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/stm32-cmake)
#     set(CMAKE_TOOLCHAIN_FILE ${STM32_CMAKE_PATH}/cmake/stm32_gcc.cmake)

#     project(FlapJack C ASM)

#     # stm32_fetch_cmsis(H7)
#     # stm32_fetch_hal(H7)
#     # Cube includes driver(cmsis and hal) and middleware (usb, freertos etc)
#     # stm32_fetch_cube(H7)
#     # stm32_find_freertos(FreeRTOS ${FREERTOS_PATH})

#     message(STATUS "Board config is: ${BOARD_CONFIG}")

#     if(NOT BOARD_CONFIG)
#         message(FATAL_ERROR "Set the BOARD_CONF environment variable.")
#     endif()

#     if("${BOARD_CONFIG}" STREQUAL "")
#         message(FATAL_ERROR "BOARD_CONF environment variable is empty!")
#     endif()

#     if(NOT "${BOARD_CONFIG}" MATCHES "^(DEV_BOARD|MY_BOARD)$")
#         message(FATAL_ERROR "Invalid BOARD_CONF: '${BOARD_CONFIG}'. Must be DEV_BOARD or MY_BOARD")
#     endif()

#     find_package(CMSIS 5.6.0 COMPONENTS STM32H747BI_M4 STM32H747BI_M7 RTOS REQUIRED) # RTOS_V2 REQUIRED)
#     find_package(HAL 1.11.5 COMPONENTS STM32H747BI_M4 STM32H747BI_M7 REQUIRED)
#     find_package(FreeRTOS COMPONENTS ARM_CM7 ARM_CM4F REQUIRED)

#     set(FJ_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Firmware/Src")
#     set(FJ_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Firmware/Inc")

#     set(FJ_SHARED_SRCS

#         ${FJ_SRC_DIR}/conf/board.c
#         ${FJ_SRC_DIR}/conf/boards/dev_board.c
#         ${FJ_SRC_DIR}/conf/boards/my_board.c

#         ${FJ_SRC_DIR}/core/core.c
#         ${FJ_SRC_DIR}/core/sync.c
#         ${FJ_SRC_DIR}/core/log/format.c
#         ${FJ_SRC_DIR}/core/log/logger.c
#         ${FJ_SRC_DIR}/core/stl/queue.c
#         ${FJ_SRC_DIR}/core/stl/ring_buff.c
#         ${FJ_SRC_DIR}/core/stl/umap.c
#         ${FJ_SRC_DIR}/core/stl/vector.c

#         ${FJ_SRC_DIR}/device/device.c
#         ${FJ_SRC_DIR}/device/flash/flash.c
#         ${FJ_SRC_DIR}/device/gps/gps.c
#         ${FJ_SRC_DIR}/device/imu/imu.c
#         ${FJ_SRC_DIR}/device/mag/mag.c
#         ${FJ_SRC_DIR}/device/motor/motor.c
#         ${FJ_SRC_DIR}/device/servo/servo.c
#         ${FJ_SRC_DIR}/device/serial/serial.c

#         ${FJ_SRC_DIR}/mc/actuators.c
#         ${FJ_SRC_DIR}/mc/dshot.c
#         ${FJ_SRC_DIR}/mc/filter.c
#         ${FJ_SRC_DIR}/mc/mc.c
#         ${FJ_SRC_DIR}/mc/pid.c

#         ${FJ_SRC_DIR}/peripheral/dma.c
#         ${FJ_SRC_DIR}/peripheral/gpio.c
#         ${FJ_SRC_DIR}/peripheral/timer.c

#         ${FJ_SRC_DIR}/peripheral/bus/bus.c
#         ${FJ_SRC_DIR}/peripheral/bus/i2c.c
#         ${FJ_SRC_DIR}/peripheral/bus/spi.c
#         ${FJ_SRC_DIR}/peripheral/bus/uart.c

#         ${FJ_SRC_DIR}/common.c
#         ${FJ_SRC_DIR}/control.c
#         ${FJ_SRC_DIR}/fcstate.c
#         ${FJ_SRC_DIR}/scheduler.c
#         ${FJ_SRC_DIR}/syscalls.c
#         ${FJ_SRC_DIR}/main.c
#         # ${FJ_SRC_DIR}/system_stm32h7xx_dualcore_boot_cm4_cm7.c
#     )

#     set(FJ_SHARED_DEFINES
#         ${BOARD_CONFIG}
#         USE_HAL_DRIVER
#         # USE_FULL_LL_DRIVER
#         STM32H747xx
#         USE_PWR_DIRECT_SMPS_SUPPLY
#         # USE_PWR_LDO_SUPPLY
#     )

#     set(FJ_SHARED_CFLAGS
#         -std=c11
#         #-pedantic
#         # -pedantic-errors
#         # -Werror
#         -Wall
#         -Wextra
#         #-Waggregate-return
#         -Wbad-function-cast
#         -Wcast-align
#         -Wcast-qual
#         -Wfloat-equal
#         -Wformat=2
#         -Winline
#         #-Wmissing-declarations
#         #-Wmissing-prototypes
#         -Wnested-externs
#         -Wpointer-arith
#         -Wredundant-decls
#         -Wshadow
#         -Wstrict-prototypes
#         -Wswitch
#         -Wundef
#         -Wunreachable-code
#         -Wunused
#         -Wwrite-strings
#         -Wstrict-aliasing
#     )
#     list(JOIN FJ_SHARED_CFLAGS " " FJ_SHARED_CFLAGS_STRING)

#     add_executable(m7core 
#         ${FJ_SRC_DIR}/cm7/freertos.c
#         # ${FJ_SRC_DIR}/cm7/stm32h7xx_hal_timebase_tim.c
#         ${FJ_SRC_DIR}/cm7/stm32h7xx_it.c
#         # ${FJ_SRC_DIR}/cm7/startup_stm32h747xihx.s
#         ${FJ_SHARED_SRCS}

#         ${CMSIS_SOURCES}
#     )

#     # message(STATUS "HAL include dirs for m7core: ${HAL_INCLUDE_DIRS}")
#     # message(STATUS "HAL sources for m7core: ${HAL_SOURCES}")
#     # message(STATUS "CMSIS include dirs for m7core: ${CMSIS_INCLUDE_DIRS}")
#     # message(STATUS "CMSIS sources for m7core: ${CMSIS_SOURCES}")
#     set_source_files_properties(${FJ_SHARED_SRCS} PROPERTIES COMPILE_FLAGS ${FJ_SHARED_CFLAGS_STRING})
#     # target_compile_options(m7core PRIVATE ${FJ_SHARED_CFLAGS})
#     target_compile_definitions(m7core PRIVATE ${FJ_SHARED_DEFINES} CORE_CM7)
#     target_include_directories(m7core PRIVATE 
#         ${FJ_INC_DIR}
#         ${FJ_INC_DIR}/cm7
#         ${CMSIS_INCLUDE_DIRS}
#         ${HAL_INCLUDE_DIRS}
#     )
#     target_link_libraries(m7core PRIVATE
#         HAL::STM32::H7::M7
#         HAL::STM32::H7::M7::RCC
#         HAL::STM32::H7::M7::RCCEx
#         HAL::STM32::H7::M7::GPIO
#         HAL::STM32::H7::M7::CORTEX
#         HAL::STM32::H7::M7::DMA
#         HAL::STM32::H7::M7::DMAEx
#         HAL::STM32::H7::M7::HSEM
#         HAL::STM32::H7::M7::UART
#         HAL::STM32::H7::M7::UARTEx
#         HAL::STM32::H7::M7::SPI
#         HAL::STM32::H7::M7::SPIEx
#         HAL::STM32::H7::M7::TIM
#         HAL::STM32::H7::M7::TIMEx
#         HAL::STM32::H7::M7::PWR
#         HAL::STM32::H7::M7::PWREx
#         FreeRTOS::ARM_CM7
#         CMSIS::STM32::H747BI::M7
#         CMSIS::STM32::H7::M7::RTOS #_V2
#         STM32::Nano
#         )
#     # Commented out stm32-cmake's autogen linker script.
#     # stm32_add_linker_script(m7core PRIVATE Scripts/cm7.ld)
#     # target_link_options(m7core PRIVATE -T "${CMAKE_CURRENT_SOURCE_DIR}/Scripts/test.ld")
#     # get_target_property(LINK_DEPENDS m7core LINK_DEPENDS)
#     # message(STATUS "Linker dependencies: ${LINK_DEPENDS}")
#     target_link_options(m7core PRIVATE 
#         "-T${CMAKE_CURRENT_SOURCE_DIR}/Scripts/cm7.ld"
#         "-Wl,--gc-sections"
#         "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/m7core.map"
#     )
#     stm32_print_size_of_target(m7core)

#     add_executable(m4core 
#         ${FJ_SRC_DIR}/cm4/freertos.c
#         # ${FJ_SRC_DIR}/cm4/stm32h7xx_hal_timebase_tim.c
#         ${FJ_SRC_DIR}/cm4/stm32h7xx_it.c
#         # ${FJ_SRC_DIR}/cm4/startup_stm32h747xihx.s
#         ${FJ_SHARED_SRCS}

#         ${CMSIS_SOURCES}
#     )
#     set_source_files_properties(${FJ_SHARED_SRCS} PROPERTIES COMPILE_FLAGS ${FJ_SHARED_CFLAGS_STRING})
#     # target_compile_options(m4core PRIVATE ${FJ_SHARED_CFLAGS})
#     target_compile_definitions(m4core PRIVATE ${FJ_SHARED_DEFINES} CORE_CM4)
#     target_include_directories(m4core PRIVATE 
#         ${FJ_INC_DIR}
#         ${FJ_INC_DIR}/cm4
#         ${CMSIS_INCLUDE_DIRS}
#         ${HAL_INCLUDE_DIRS}
#     )
#     target_link_libraries(m4core PRIVATE
#         HAL::STM32::H7::M4
#         HAL::STM32::H7::M4::RCC
#         HAL::STM32::H7::M4::RCCEx
#         HAL::STM32::H7::M4::GPIO
#         HAL::STM32::H7::M4::CORTEX
#         HAL::STM32::H7::M4::HSEM
#         HAL::STM32::H7::M4::DMA
#         HAL::STM32::H7::M4::DMAEx
#         HAL::STM32::H7::M4::UART
#         HAL::STM32::H7::M4::UARTEx
#         HAL::STM32::H7::M4::SPI
#         HAL::STM32::H7::M4::SPIEx
#         HAL::STM32::H7::M4::TIM
#         HAL::STM32::H7::M4::TIMEx
#         HAL::STM32::H7::M4::PWR
#         HAL::STM32::H7::M4::PWREx
#         FreeRTOS::ARM_CM4F
#         CMSIS::STM32::H747BI::M4
#         CMSIS::STM32::H7::M4::RTOS #_V2
#         STM32::Nano
#         )
#     # stm32_add_linker_script(m4core PRIVATE Scripts/cm4.ld)
#     target_link_options(m4core PRIVATE 
#         "-T${CMAKE_CURRENT_SOURCE_DIR}/Scripts/cm4.ld"
#         "-Wl,--gc-sections"
#         "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/m4core.map"
#     )
#     stm32_print_size_of_target(m4core)
    
# else()

#     project(FlapJackTests C)
#     set(CMAKE_C_STANDARD 11)

#     set(FJ_TESTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Tests")
#     set(FJ_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Common/Src")
#     set(FJ_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Common/Inc")

#     set(CMOCK_OUTPUT_BASE_DIR "${FJ_TESTS_DIR}/mocks")
#     set(CMOCK_CONFIG_FILE "${FJ_TESTS_DIR}/cmock_config.yml")

#     set(STM32_HAL_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc")
#     set(STM32_CMSIS_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include")
#     set(STM32_CMSIS_CORE_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include")

#     # Add my cmake modules to the path
#     list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

#     include (tests)

# endif()