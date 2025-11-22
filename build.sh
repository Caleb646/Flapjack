#!/bin/bash

# echo "Checking toolchain file..."
# if [ ! -f "./cmake/gcc_tc.cmake" ]; then
#     echo "ERROR: Toolchain file not found!"
#     exit 1
# fi

# cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_TOOLCHAIN_FILE=./cmake/gcc_tc.cmake -DBUILD_TESTS=ON
# cmake --build build # --verbose 2>&1 | tee ./build/build_log.txt

# Append mingw to the front of the path to ensure the correct gcc/g++ are used
export PATH="/c/msys64/ucrt64/bin:$PATH"

cmake -S . -B build \
    -DCMAKE_C_COMPILER=gcc \
    -DCMAKE_CXX_COMPILER=g++ \
    -G "MinGW Makefiles" \
    -DBUILD_TESTS=ON \
    -DCMAKE_BUILD_TYPE=Debug

cmake --build build


# ctest -V --test-dir build -C Debug --output-on-failure
ctest --test-dir build -C Debug --output-on-failure