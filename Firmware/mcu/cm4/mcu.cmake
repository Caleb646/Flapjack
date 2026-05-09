# set(CM4_ROOT_DIR "${FLAPJACK_ROOT}/CM4")
set(CM4_INC_DIR "${CM4_ROOT}/inc")
set(CM4_SRC_DIR "${CM4_ROOT}/src")

set(CM4_INCS ${CM4_INC_DIR})
file(GLOB_RECURSE CM4_SRCS
    "${CM4_SRC_DIR}/*.c"
    "${CM4_SRC_DIR}/*.s"
)

set(CM4_DEFINES CORE_CM4)

set(CM4_COMPILER_OPTIONS
    # use C11 standard with GNU extensions
    -std=gnu11
    # syscalls are simple stubs that return an error
    --specs=nosys.specs
    # use newlib nano for smaller footprint
    --specs=nano.specs
    -mcpu=cortex-m4
    -mthumb 
    -mfpu=fpv4-sp-d16 
    -mfloat-abi=hard 
    -fdata-sections 
    -ffunction-sections
)

set(CM4_LINKER_SCRIPT "${CM4_ROOT}/cm4_flash.ld")

set(CM4_LINKER_OPTIONS 
    -mcpu=cortex-m4 
    -mfpu=fpv4-sp-d16 
    -mfloat-abi=hard
    -fdata-sections 
    -ffunction-sections
    -static
    -mthumb
    --specs=nosys.specs
    --specs=nano.specs
    -T${CM4_LINKER_SCRIPT}
    -Wl,--gc-sections
    -Wl,--start-group -lc -lm -Wl,--end-group
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/cm4.map
)

list(APPEND TARGETS cm4)
add_executable(cm4 
    ${STM32_SRCS}
    ${FIRMWARE_SRCS}
    ${CM4_SRCS}
)

set_source_files_properties(${FIRMWARE_SRCS} PROPERTIES COMPILE_FLAGS ${CFLAGS_STRING})

target_compile_options(cm4 PRIVATE 
    ${CM4_COMPILER_OPTIONS}
)

target_link_options(cm4 PRIVATE 
    ${CM4_LINKER_OPTIONS}
)

target_compile_definitions(cm4 PRIVATE
    ${CM4_DEFINES}
    ${BOARD_DEFINES}
    ${STM32_DEFINES}
    ${FIRMWARE_DEFINES}
)

# use SYSTEM for cmsis and hal includes to suppress warnings from CMSIS and HAL
target_include_directories(cm4 SYSTEM PRIVATE
    ${STM32_INCS}
    ${CM4_INCS}
)

target_include_directories(cm4 PRIVATE
    ${FIRMWARE_INCS}
    ${TARGET_INCS}
)