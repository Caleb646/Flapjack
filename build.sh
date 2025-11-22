#!/bin/bash

# echo "Checking toolchain file..."
# if [ ! -f "./cmake/gcc_tc.cmake" ]; then
#     echo "ERROR: Toolchain file not found!"
#     exit 1
# fi

# cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_TOOLCHAIN_FILE=./cmake/gcc_tc.cmake -DBUILD_TESTS=ON
# cmake --build build # --verbose 2>&1 | tee ./build/build_log.txt

# Append mingw to the front of the path to ensure the correct gcc/g++ are used
# export PATH="/c/msys64/ucrt64/bin:$PATH"

# cmake -S . -B build/tests \
#     -DCMAKE_C_COMPILER=gcc \
#     -DCMAKE_CXX_COMPILER=g++ \
#     -G "MinGW Makefiles" \
#     -DBUILD_TESTS=ON \
#     -DCMAKE_BUILD_TYPE=Debug

# cmake --build build/tests


# # ctest -V --test-dir build/tests -C Debug --output-on-failure
# ctest --test-dir build/tests -C Debug --output-on-failure


# download: https://developer.arm.com/-/media/Files/downloads/gnu/14.3.rel1/binrel/arm-gnu-toolchain-14.3.rel1-mingw-w64-x86_64-arm-none-eabi.zip
export STM32_TOOLCHAIN_PATH="C:/arm_none_eabi_14.3.rel1"
export STM32_TARGET_TRIPLET=arm-none-eabi
# Note: these wont exist until cmake fetches and downloads stm32 cube
export STM32_CUBE_H7_PATH=${PWD}"/build/stm32/_deps/stm32cubeh7-src"
export FREERTOS_PATH=${PWD}"/build/stm32/_deps/stm32cubeh7-src/Middlewares/Third_Party"

if [ ! -d "${STM32_TOOLCHAIN_PATH}" ]; then
    echo "${STM32_TOOLCHAIN_PATH} does not exist."
    exit 1
fi

cmake --fresh -S . -B build/stm32 -G "MinGW Makefiles"
cmake --build build/stm32