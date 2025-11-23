cmake_minimum_required(VERSION 3.16)


if(NOT BUILD_TESTS)

    set(STM32_CMAKE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/stm32-cmake)
    set(CMAKE_TOOLCHAIN_FILE ${STM32_CMAKE_PATH}/cmake/stm32_gcc.cmake)

    project(FlapJack C ASM)

    # stm32_fetch_cmsis(H7)
    # stm32_fetch_hal(H7)
    # Cube includes driver(cmsis and hal) and middleware (usb, freertos etc)
    stm32_fetch_cube(H7)
    # stm32_find_freertos(FreeRTOS ${FREERTOS_PATH})

    if(NOT DEFINED ENV{BOARD_CONF})
        message(FATAL_ERROR "Set the BOARD_CONF environment variable.")
    endif()

    if("$ENV{BOARD_CONF}" STREQUAL "")
        message(FATAL_ERROR "BOARD_CONF environment variable is empty!")
    endif()

    if(NOT "$ENV{BOARD_CONF}" MATCHES "^(DEV_BOARD|MY_BOARD)$")
        message(FATAL_ERROR "Invalid BOARD_CONF: '$ENV{BOARD_CONF}'. Must be DEV_BOARD or MY_BOARD")
    endif()

    find_package(CMSIS 5.6.0 COMPONENTS STM32H747BI_M4 STM32H747BI_M7 RTOS REQUIRED) # RTOS_V2 REQUIRED)
    find_package(HAL 1.11.5 COMPONENTS STM32H747BI_M4 STM32H747BI_M7 REQUIRED)
    find_package(FreeRTOS COMPONENTS ARM_CM7 ARM_CM4F REQUIRED)

    set(FJ_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Firmware/Src")
    set(FJ_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Firmware/Inc")

    set(FJ_SHARED_SRCS

        ${FJ_SRC_DIR}/conf/board.c
        ${FJ_SRC_DIR}/conf/boards/dev_board.c
        ${FJ_SRC_DIR}/conf/boards/my_board.c

        ${FJ_SRC_DIR}/core/core.c
        ${FJ_SRC_DIR}/core/sync.c
        ${FJ_SRC_DIR}/core/log/format.c
        ${FJ_SRC_DIR}/core/log/logger.c
        ${FJ_SRC_DIR}/core/stl/queue.c
        ${FJ_SRC_DIR}/core/stl/ring_buff.c
        ${FJ_SRC_DIR}/core/stl/umap.c
        ${FJ_SRC_DIR}/core/stl/vector.c

        ${FJ_SRC_DIR}/device/device.c
        ${FJ_SRC_DIR}/device/flash/flash.c
        ${FJ_SRC_DIR}/device/gps/gps.c
        ${FJ_SRC_DIR}/device/imu/imu.c
        ${FJ_SRC_DIR}/device/mag/mag.c
        ${FJ_SRC_DIR}/device/motor/motor.c
        ${FJ_SRC_DIR}/device/servo/servo.c
        ${FJ_SRC_DIR}/device/serial/serial.c

        ${FJ_SRC_DIR}/mc/actuators.c
        ${FJ_SRC_DIR}/mc/dshot.c
        ${FJ_SRC_DIR}/mc/filter.c
        ${FJ_SRC_DIR}/mc/mc.c
        ${FJ_SRC_DIR}/mc/pid.c

        ${FJ_SRC_DIR}/peripheral/dma.c
        ${FJ_SRC_DIR}/peripheral/gpio.c
        ${FJ_SRC_DIR}/peripheral/timer.c

        ${FJ_SRC_DIR}/peripheral/bus/bus.c
        ${FJ_SRC_DIR}/peripheral/bus/i2c.c
        ${FJ_SRC_DIR}/peripheral/bus/spi.c
        ${FJ_SRC_DIR}/peripheral/bus/uart.c

        ${FJ_SRC_DIR}/common.c
        ${FJ_SRC_DIR}/control.c
        ${FJ_SRC_DIR}/fcstate.c
        ${FJ_SRC_DIR}/main.c
        # ${FJ_SRC_DIR}/system_stm32h7xx_dualcore_boot_cm4_cm7.c
    )

    set(FJ_SHARED_DEFINES
        $ENV{BOARD_CONF}
        USE_HAL_DRIVER
        STM32H747xx
        USE_PWR_LDO_SUPPLY
    )

    add_executable(m7core 
        ${FJ_SRC_DIR}/cm7/freertos.c
        # ${FJ_SRC_DIR}/cm7/stm32h7xx_hal_timebase_tim.c
        ${FJ_SRC_DIR}/cm7/stm32h7xx_it.c
        # ${FJ_SRC_DIR}/cm7/startup_stm32h747xihx.s
        ${FJ_SHARED_SRCS}

        ${CMSIS_SOURCES}
    )
    # message(STATUS "HAL include dirs for m7core: ${HAL_INCLUDE_DIRS}")
    # message(STATUS "HAL sources for m7core: ${HAL_SOURCES}")
    # message(STATUS "CMSIS include dirs for m7core: ${CMSIS_INCLUDE_DIRS}")
    # message(STATUS "CMSIS sources for m7core: ${CMSIS_SOURCES}")
    
    target_compile_definitions(m7core PRIVATE ${FJ_SHARED_DEFINES} CORE_CM7)
    target_include_directories(m7core PRIVATE 
        ${FJ_INC_DIR}
        ${FJ_INC_DIR}/cm7
        ${CMSIS_INCLUDE_DIRS}
        ${HAL_INCLUDE_DIRS}
    )
    target_link_libraries(m7core PRIVATE
        HAL::STM32::H7::M7
        HAL::STM32::H7::M7::RCC
        HAL::STM32::H7::M7::RCCEx
        HAL::STM32::H7::M7::GPIO
        HAL::STM32::H7::M7::CORTEX
        HAL::STM32::H7::M7::DMA
        HAL::STM32::H7::M7::DMAEx
        HAL::STM32::H7::M7::UART
        HAL::STM32::H7::M7::UARTEx
        HAL::STM32::H7::M7::SPI
        HAL::STM32::H7::M7::SPIEx
        HAL::STM32::H7::M7::TIM
        HAL::STM32::H7::M7::TIMEx
        HAL::STM32::H7::M7::PWR
        HAL::STM32::H7::M7::PWREx
        FreeRTOS::ARM_CM7
        CMSIS::STM32::H747BI::M7
        CMSIS::STM32::H7::M7::RTOS #_V2
        STM32::Nano
        )
    # Commented out stm32-cmake's autogen linker script.
    # stm32_add_linker_script(m7core PRIVATE Scripts/cm7.ld)
    # target_link_options(m7core PRIVATE -T "${CMAKE_CURRENT_SOURCE_DIR}/Scripts/test.ld")
    # get_target_property(LINK_DEPENDS m7core LINK_DEPENDS)
    # message(STATUS "Linker dependencies: ${LINK_DEPENDS}")
    target_link_options(m7core PRIVATE 
        "-T${CMAKE_CURRENT_SOURCE_DIR}/Scripts/cm7.ld"
        "-Wl,--gc-sections"
        "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/m7core.map"
    )
    stm32_print_size_of_target(m7core)

    add_executable(m4core 
        ${FJ_SRC_DIR}/cm4/freertos.c
        # ${FJ_SRC_DIR}/cm4/stm32h7xx_hal_timebase_tim.c
        ${FJ_SRC_DIR}/cm4/stm32h7xx_it.c
        # ${FJ_SRC_DIR}/cm4/startup_stm32h747xihx.s
        ${FJ_SHARED_SRCS}

        ${CMSIS_SOURCES}
    )
    target_compile_definitions(m4core PRIVATE ${FJ_SHARED_DEFINES} CORE_CM4)
    target_include_directories(m4core PRIVATE 
        ${FJ_INC_DIR}
        ${FJ_INC_DIR}/cm4
        ${CMSIS_INCLUDE_DIRS}
        ${HAL_INCLUDE_DIRS}
    )
    target_link_libraries(m4core PRIVATE
        HAL::STM32::H7::M4
        HAL::STM32::H7::M4::RCC
        HAL::STM32::H7::M4::RCCEx
        HAL::STM32::H7::M4::GPIO
        HAL::STM32::H7::M4::CORTEX
        HAL::STM32::H7::M4::DMA
        HAL::STM32::H7::M4::DMAEx
        HAL::STM32::H7::M4::UART
        HAL::STM32::H7::M4::UARTEx
        HAL::STM32::H7::M4::SPI
        HAL::STM32::H7::M4::SPIEx
        HAL::STM32::H7::M4::TIM
        HAL::STM32::H7::M4::TIMEx
        HAL::STM32::H7::M4::PWR
        HAL::STM32::H7::M4::PWREx
        FreeRTOS::ARM_CM4F
        CMSIS::STM32::H747BI::M4
        CMSIS::STM32::H7::M4::RTOS #_V2
        STM32::Nano
        )
    # stm32_add_linker_script(m4core PRIVATE Scripts/cm4.ld)
    target_link_options(m4core PRIVATE 
        "-T${CMAKE_CURRENT_SOURCE_DIR}/Scripts/cm4.ld"
        "-Wl,--gc-sections"
        "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/m4core.map"
    )
    stm32_print_size_of_target(m4core)
    
else()

    project(FlapJackTests C)
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

    include (tests)

endif()