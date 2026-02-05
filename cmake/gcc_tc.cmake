if(${CMAKE_VERSION} VERSION_LESS "3.16.0") 
    message(WARNING "Current CMake version is ${CMAKE_VERSION}. stm32-cmake requires CMake 3.16 or greater")
endif()

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Specify the cross compiler
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

# This should be safe to set for a bare-metal cross-compiler
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)