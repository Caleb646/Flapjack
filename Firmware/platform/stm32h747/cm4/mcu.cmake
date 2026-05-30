set(CM4_INC_DIR "${CM4_ROOT}/inc")
set(CM4_SRC_DIR "${CM4_ROOT}/src")

set(CM4_INCS ${CM4_INC_DIR} "${FREERTOS_ROOT}/portable/GCC/ARM_CM4F")
file(GLOB_RECURSE CM4_SRCS
    "${CM4_SRC_DIR}/*.c"
    "${CM4_SRC_DIR}/*.s"
)
list(APPEND CM4_SRCS
    "${FREERTOS_ROOT}/portable/GCC/ARM_CM4F/port.c"
    "${FREERTOS_ROOT}/portable/MemMang/heap_1.c"
)

add_executable(cm4 ${CM4_SRCS})
list(APPEND TARGETS cm4)

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

target_compile_options(cm4 PRIVATE
    ${CM4_COMPILER_OPTIONS}
)

target_link_options(cm4 PRIVATE
    ${CM4_LINKER_OPTIONS}
)

target_compile_definitions(cm4 PRIVATE CORE_CM4)

target_include_directories(cm4 SYSTEM PRIVATE ${CM4_INCS})
