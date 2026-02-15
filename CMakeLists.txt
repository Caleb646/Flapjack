cmake_minimum_required(VERSION 3.16)

######## Toolchain setup start ###########

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

find_program(CMAKE_C_COMPILER arm-none-eabi-gcc HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_CXX_COMPILER arm-none-eabi-g++ HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_ASM_COMPILER arm-none-eabi-gcc HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_AR arm-none-eabi-ar HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_OBJDUMP arm-none-eabi-objdump HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_SIZE arm-none-eabi-size HINTS ${TOOLCHAIN_BIN_PATH})

get_filename_component(STM32_CMAKE_DIR ${CMAKE_CURRENT_LIST_FILE} DIRECTORY)
list(APPEND CMAKE_MODULE_PATH ${STM32_CMAKE_DIR})

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_EXECUTABLE_SUFFIX_C   .elf)
set(CMAKE_EXECUTABLE_SUFFIX_CXX .elf)
set(CMAKE_EXECUTABLE_SUFFIX_ASM .elf)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

######## Toolchain setup end ###########

project(FlapJack C ASM)

set(FJ_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
include(cmake/board.cmake)
include(cmake/stm32.cmake)
include(cmake/firmware.cmake)

message(STATUS "Building for board: ${BOARD_NAME}")
message(STATUS "Board defines: ${BOARD_DEFINES}")

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

##################### CM7 Core Build ########################

add_executable(m7core 
    ${STM32_SRCS}
    ${FW_COMMON_SRCS}
    ${FW_CM7_SRCS}
)

# add_custom_command(TARGET m7core POST_BUILD
#     COMMAND ${CMAKE_SIZE} -A $<TARGET_FILE:m7core>
#     COMMENT "Size information for m7core"
# )

list(JOIN FJ_SHARED_CFLAGS " " FJ_SHARED_CFLAGS_STRING)
set_source_files_properties(${FW_CM7_SRCS} PROPERTIES COMPILE_FLAGS ${FJ_SHARED_CFLAGS_STRING})
set_source_files_properties(${FW_COMMON_SRCS} PROPERTIES COMPILE_FLAGS ${FJ_SHARED_CFLAGS_STRING})

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
    -T${FW_CM7_LINKER_SCRIPT}
    -Wl,--gc-sections
    -Wl,--start-group -lc -lm -Wl,--end-group
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/m7core.map
)

target_compile_definitions(m7core PRIVATE
    ${STM32_DEFINES}
    ${FW_CM7_DEFINES}
    ${BOARD_DEFINES}
)

# use SYSTEM for cmsis and hal includes to suppress warnings from CMSIS and HAL
target_include_directories(m7core SYSTEM PRIVATE
    ${STM32_INCS}
)
target_include_directories(m7core PRIVATE
    ${FW_COMMON_INCS}
    ${FW_CM7_INCS}
)


##################### CM4 Core Build ########################

add_executable(m4core 
    ${STM32_SRCS}
    ${FW_COMMON_SRCS}
    ${FW_CM4_SRCS}
)

# add_custom_command(TARGET m4core POST_BUILD
#     COMMAND ${CMAKE_SIZE} -A $<TARGET_FILE:m4core>
#     COMMENT "Size information for m4core"
# )

set_source_files_properties(${FW_CM4_SRCS} PROPERTIES COMPILE_FLAGS ${FJ_SHARED_CFLAGS_STRING})

target_compile_options(m4core PRIVATE 
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
target_link_options(m4core PRIVATE 
    -mcpu=cortex-m4 
    -mfpu=fpv4-sp-d16 
    -mfloat-abi=hard
    -fdata-sections 
    -ffunction-sections
    -static
    -mthumb
    --specs=nosys.specs
    --specs=nano.specs
    -T${FW_CM4_LINKER_SCRIPT}
    -Wl,--gc-sections
    -Wl,--start-group -lc -lm -Wl,--end-group
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/m4core.map
)

target_compile_definitions(m4core PRIVATE
    ${STM32_DEFINES}
    ${FW_CM4_DEFINES}
    ${BOARD_DEFINES}
)
# use SYSTEM for cmsis and hal includes to suppress warnings from CMSIS and HAL
target_include_directories(m4core SYSTEM PRIVATE
    ${STM32_INCS}
)
target_include_directories(m4core PRIVATE
    ${FW_COMMON_INCS}
    ${FW_CM4_INCS}
)


##################################################

add_custom_target(size ALL
    DEPENDS m7core m4core
    COMMAND ${CMAKE_SIZE} -A $<TARGET_FILE:m7core>
    COMMAND ${CMAKE_SIZE} -A $<TARGET_FILE:m4core>
    COMMENT "Size information for both cores"
)