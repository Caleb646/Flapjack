#!/bin/bash

# echo "Checking toolchain file..."
# if [ ! -f "./cmake/gcc_tc.cmake" ]; then
#     echo "ERROR: Toolchain file not found!"
#     exit 1
# fi

# cmake -S . -B Build -G "MinGW Makefiles" -DCMAKE_TOOLCHAIN_FILE=./cmake/gcc_tc.cmake -DBUILD_TESTS=ON
# cmake --build Build # --verbose 2>&1 | tee ./Build/build_log.txt

# Append mingw to the front of the path to ensure the correct gcc/g++ are used
# export PATH="/c/msys64/ucrt64/bin:$PATH"

# cmake -S . -B Build/tests \
#     -DCMAKE_C_COMPILER=gcc \
#     -DCMAKE_CXX_COMPILER=g++ \
#     -G "MinGW Makefiles" \
#     -DBUILD_TESTS=ON \
#     -DCMAKE_BUILD_TYPE=Debug

# cmake --build Build/tests


# # ctest -V --test-dir Build/tests -C Debug --output-on-failure
# ctest --test-dir Build/tests -C Debug --output-on-failure


# download: https://developer.arm.com/-/media/Files/downloads/gnu/14.3.rel1/binrel/arm-gnu-toolchain-14.3.rel1-mingw-w64-x86_64-arm-none-eabi.zip
export STM32_TOOLCHAIN_PATH="C:/arm_none_eabi_14.3.rel1"
export STM32_TARGET_TRIPLET=arm-none-eabi
# Note: these wont exist until cmake fetches and downloads stm32 cube
export STM32_CUBE_H7_PATH=${PWD}"/Build/stm32/_deps/stm32cubeh7-src"
# export CUBE_H7_VERSION="v1.12.1"
export FREERTOS_PATH=${PWD}"/Build/stm32/_deps/stm32cubeh7-src/Middlewares/Third_Party"

if [ ! -d "${STM32_TOOLCHAIN_PATH}" ]; then
    echo "${STM32_TOOLCHAIN_PATH} does not exist."
    exit 1
fi

if [ $# -eq 0 ]; then
    echo "Usage: $0 <BOARD_CONFIG>"
    echo "Available board configurations:"
    echo "  DEV_BOARD - Development board configuration"
    echo "  MY_BOARD  - Custom board configuration"
    exit 1
fi

export BOARD_CONF=$1

cmake --fresh -S . -B Build/stm32 -G "MinGW Makefiles"
cmake --build Build/stm32