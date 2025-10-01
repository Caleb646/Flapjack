#!/bin/bash

cmake -S . -B build # -DCMAKE_C_COMPILER=arm-none-eabi-gcc -DCMAKE_CXX_COMPILER=arm-none-eabi-g++
cmake --build build # --verbose 2>&1 | tee ./build/build_log.txt
# ./build/Debug/test_queue.exe
# ctest --test-dir build
ctest --test-dir build -C Debug --output-on-failure
