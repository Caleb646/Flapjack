#!/bin/bash

cmake -S . -B build
cmake --build build
./build/Debug/test_imu.exe
# ctest --test-dir build