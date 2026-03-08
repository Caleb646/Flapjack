set(CMSIS_ROOT_DIR "${FJ_ROOT_DIR}/Vendor/STM32H7/CMSIS")
set(CMSIS_INC_DIR "${CMSIS_ROOT_DIR}/Include")
set(CMSIS_DEVICE_INC_DIR "${CMSIS_ROOT_DIR}/Device/ST/STM32H7xx/Include")

set(CMSIS_INCS
    ${CMSIS_INC_DIR}
    ${CMSIS_DEVICE_INC_DIR}
)

set(CMSIS_SRCS
    # Add startup and system files as needed
    # ${STARTUP_FILE}
    # ${BOOT_FILE}
)

set(STM32_HAL_ROOT_DIR "${FJ_ROOT_DIR}/Vendor/STM32H7/STM32H7xx_HAL_Driver")
set(STM32_HAL_INC_DIR "${STM32_HAL_ROOT_DIR}/Inc")
set(STM32_HAL_SRC_DIR "${STM32_HAL_ROOT_DIR}/Src")

set(STM32_HAL_INCS 
    ${STM32_HAL_INC_DIR}
)

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

set(STM32_INCS
    ${CMSIS_INCS}
    ${STM32_HAL_INCS}

    # PARENT_SCOPE
)

set(STM32_SRCS
    ${CMSIS_SRCS}
    ${STM32_HAL_SRCS}

    # PARENT_SCOPE
)

set(STM32_DEFINES
    USE_HAL_DRIVER
    USE_FULL_LL_DRIVER
    STM32H747xx
)