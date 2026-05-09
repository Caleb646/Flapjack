set(TARGET_INCS ${TARGET_ROOT})
set(TARGET_SRCS "")

if(BOARD_NAME STREQUAL "nucleo-h747zi")

    include("${TARGET_ROOT}/nucleo_h747zi/board.cmake")
    include("${MCU_ROOT}/stm32h747/mcu.cmake")

elseif(BOARD_NAME STREQUAL "flapjack-v1")

    include("${TARGET_ROOT}/flapjack_v1/board.cmake")
    include("${MCU_ROOT}/stm32h747/mcu.cmake")

else()

    message(ERROR "No board defined")

endif()