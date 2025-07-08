# Test System Documentation

This directory contains a comprehensive test suite for the drone project, including an automatic test runner that can discover and run tests without manual registration.

## Quick Start

From the `Tests/` directory, run:

    cmake -S . -B build
    cmake --build build
    ctest --test-dir build

Or, to run the test executable directly (after building):

    ./build/Debug/test_imu.exe

## Test Structure

All test functions should follow this naming convention:
```c
void test_FunctionName(void) {
    // Test implementation
    TEST_ASSERT_EQUAL(expected, actual);
}
```
## Automatic Test Runner

The test runner automatically discovers and runs all test functions. It provides:

1. **Automatic Registration**: Tests are automatically registered without manual declarations
2. **Progress Reporting**: Shows current test progress (X/Y: test_name)
3. **Comprehensive Coverage**: Runs all discovered tests
4. **Easy Maintenance**: Adding new tests requires no changes to the runner

### Usage

```c
// In your test file, just define test functions:
void test_MyNewFeature(void) {
    // Test implementation
}

// The runner will automatically find and run it
```

## Python Test Generator

The `register_tests.py` script can automatically scan test files and generate registration code:

```bash
python register_tests.py
```

This will:
1. Find all test files (`test_*.c`)
2. Extract test function names
3. Generate registration code
4. Output the code to `registered_tests.c`

## Building and Running Tests

1. Ensure `UNIT_TEST` is defined during compilation
2. Link with Unity framework
3. Include the Common source files for the components being tested
4. Run the executable

Example build command:
```bash
gcc -DUNIT_TEST -I../Common/Inc -I../Common/Src -Iunity \
    test_runner.c test_queue.c test_imu.c test_filter.c \
    ../Common/Src/mem/queue.c \
    unity/unity.c \
    -o test_runner
```
The tests are automatically rediscovered for every CMake build.
```bash
cmake -S . -B build
cmake --build build
./build/Debug/test_runner.exe
```

---
