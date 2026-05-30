set(CM7_INC_DIR "${CM7_ROOT}/inc")
set(CM7_SRC_DIR "${CM7_ROOT}/src")

set(CM7_INCS ${CM7_INC_DIR} "${FREERTOS_ROOT}/portable/GCC/ARM_CM7/r0p1")
file(GLOB_RECURSE CM7_SRCS
    "${CM7_SRC_DIR}/*.c"
    "${CM7_SRC_DIR}/*.s"
)
list(APPEND CM7_SRCS
    "${FREERTOS_ROOT}/portable/GCC/ARM_CM7/r0p1/port.c"
    "${FREERTOS_ROOT}/portable/MemMang/heap_1.c"
)

add_executable(cm7 ${CM7_SRCS})
list(APPEND TARGETS cm7)

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

target_compile_definitions(cm7 PRIVATE CORE_CM7)

target_include_directories(cm7 SYSTEM PRIVATE ${CM7_INCS})
