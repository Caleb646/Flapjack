if(NOT DEFINED BOARD_NAME)
    message(FATAL_ERROR "BOARD_NAME must be specified. Use -DBOARD_NAME=<board>")
endif()

if(BOARD_NAME STREQUAL "nucleo-h747zi")
    set(BOARD_DEFINES 
        BOARD_NUCLEO_H747ZI
        USE_PWR_DIRECT_SMPS_SUPPLY
    )
elseif(BOARD_NAME STREQUAL "flapjack-v1")
    set(BOARD_DEFINES
        BOARD_FLAPJACK_V1
        USE_PWR_LDO_SUPPLY
        HSE_VALUE=0
    )
endif()