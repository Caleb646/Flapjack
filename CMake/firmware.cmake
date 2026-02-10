set(FW_ROOT_DIR "${FJ_ROOT_DIR}/Firmware")
set(FW_COMMON_INC_DIR "${FW_ROOT_DIR}/Common/Inc")
set(FW_COMMON_SRC_DIR "${FW_ROOT_DIR}/Common/Src")

set(FW_COMMON_INCS
    ${FW_COMMON_INC_DIR}
    ${FW_ROOT_DIR}/Targets

    # PARENT_SCOPE
)

file(GLOB_RECURSE FW_COMMON_SRCS
    "${FW_COMMON_SRC_DIR}/*.c"
)
set(FW_COMMON_SRCS 
    ${FW_COMMON_SRCS} 

    # PARENT_SCOPE
)

######### CM7 ####################

set(FW_CM7_ROOT_DIR "${FW_ROOT_DIR}/CM7")
set(FW_CM7_INC_DIR "${FW_CM7_ROOT_DIR}/Core/Inc")
set(FW_CM7_SRC_DIR "${FW_CM7_ROOT_DIR}/Core/Src")

set(FW_CM7_INCS
    ${FW_CM7_INC_DIR}

    # PARENT_SCOPE
)


file(GLOB_RECURSE FW_CM7_SRCS
    "${FW_CM7_SRC_DIR}/*.c"
    "${FW_CM7_SRC_DIR}/*.s"
)

set(FW_CM7_DEFINES
    CORE_CM7

    # PARENT_SCOPE
) 

set(FW_CM7_LINKER_SCRIPT 
    "${FW_CM7_ROOT_DIR}/CM7_Flash.ld" 
    
    # PARENT_SCOPE
)

################ CM4 ##########################

set(FW_CM4_ROOT_DIR "${FW_ROOT_DIR}/CM4")
set(FW_CM4_INC_DIR "${FW_CM4_ROOT_DIR}/Core/Inc")
set(FW_CM4_SRC_DIR "${FW_CM4_ROOT_DIR}/Core/Src")

set(FW_CM4_INCS
    ${FW_CM4_INC_DIR}

    # PARENT_SCOPE
)

file(GLOB_RECURSE FW_CM4_SRCS
    "${FW_CM4_SRC_DIR}/*.c"
    "${FW_CM4_SRC_DIR}/*.s"
)

set(FW_CM4_DEFINES
    CORE_CM4

    # PARENT_SCOPE
)

set(FW_CM4_LINKER_SCRIPT 
    "${FW_CM4_ROOT_DIR}/CM4_Flash.ld" 

    #PARENT_SCOPE
)