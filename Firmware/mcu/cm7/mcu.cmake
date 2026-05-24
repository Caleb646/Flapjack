# set(CM7_ROOT_DIR "${FLAPJACK_ROOT}/CM7")
set(CM7_INC_DIR "${CM7_ROOT}/inc")
set(CM7_SRC_DIR "${CM7_ROOT}/src")

set(CM7_INCS ${CM7_INC_DIR})
file(GLOB_RECURSE CM7_SRCS
    "${CM7_SRC_DIR}/*.c"
    "${CM7_SRC_DIR}/*.s"
)

list(APPEND TARGETS cm7)
add_executable(cm7
    ${CM7_SRCS}
    ${STM32_SRCS}
    ${FIRMWARE_SRCS}
    ${NANOPB_SRCS}
)

set_source_files_properties(${FIRMWARE_SRCS} PROPERTIES COMPILE_FLAGS ${CFLAGS_STRING})

set(CM7_DEFINES CORE_CM7)

set(CM7_COMPILER_OPTIONS
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

set(CM7_LINKER_SCRIPT "${CM7_ROOT}/cm7_flash.ld")

set(CM7_LINKER_OPTIONS 
    -mcpu=cortex-m7 
    -mfpu=fpv5-sp-d16 
    -mfloat-abi=hard
    -fdata-sections 
    -ffunction-sections
    -static
    -mthumb
    --specs=nosys.specs
    --specs=nano.specs
    -T${CM7_LINKER_SCRIPT}
    -Wl,--gc-sections
    -Wl,--start-group -lc -lm -Wl,--end-group
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/cm7.map
)

target_compile_options(cm7 PRIVATE 
    ${CM7_COMPILER_OPTIONS}
)

target_link_options(cm7 PRIVATE 
    ${CM7_LINKER_OPTIONS}
)

target_compile_definitions(cm7 PRIVATE
    ${CM7_DEFINES}
    ${BOARD_DEFINES}
    ${STM32_DEFINES}
    ${FIRMWARE_DEFINES}
)

# use SYSTEM for cmsis and hal includes to suppress warnings from CMSIS and HAL
target_include_directories(cm7 SYSTEM PRIVATE
    ${STM32_INCS}
    ${CM7_INCS}
)

target_include_directories(cm7 PRIVATE
    ${FIRMWARE_INCS}
    ${TARGET_INCS}
)

target_include_directories(cm7 SYSTEM PRIVATE
    ${NANOPB_ROOT}
    ${PROTO_GEN_DIR}
)