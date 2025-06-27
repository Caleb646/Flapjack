# Running tests
From the `Tests/` directory, run:

    cmake -S . -B build
    cmake --build build
    ctest --test-dir build

Or, to run the test executable directly (after building):

    ./build/Debug/test_imu.exe

---
