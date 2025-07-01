#!/bin/bash

cmake -S . -B build
cmake --build build
./build/Debug/test_runner.exe
# ctest --test-dir build