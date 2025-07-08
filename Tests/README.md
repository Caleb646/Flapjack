# Test System Documentation

This directory contains a comprehensive test suite for the drone project, including an automatic test runner that can discover and run tests without manual registration.

## Quick Start

From the `Tests/` directory, run:

    cmake -S . -B build
    cmake --build build
    ctest --test-dir build

Or, to run the test executable directly (after building):

    ./build/Debug/test_imu.exe

## Files Overview

- `test_runner.c` - Enhanced test runner with automatic test discovery
- `test_runner.h` - Header file for the test runner
- `test_runner_auto.c` - Alternative test runner implementation
- `test_queue.c` - Comprehensive tests for the queue implementation
- `test_imu.c` - IMU sensor tests
- `test_filter.c` - Madgwick filter tests
- `generate_test_registrations.py` - Python script to automatically generate test registrations

## Test Structure

All test functions should follow this naming convention:
```c
void test_FunctionName(void) {
    // Test implementation
    TEST_ASSERT_EQUAL(expected, actual);
}
```

## Queue Tests (`test_queue.c`)

Comprehensive tests for the generic queue implementation covering:

### Basic Functionality
- `test_QueueInit_ValidParameters` - Valid initialization
- `test_QueueEnqueueDequeue_Basic` - Basic enqueue/dequeue operations
- `test_QueueFIFOOrdering` - FIFO ordering verification
- `test_QueueCircularBehavior` - Circular buffer behavior

### Error Handling
- `test_QueueInit_NullQueue` - NULL queue pointer
- `test_QueueInit_NullBuffer` - NULL buffer pointer
- `test_QueueInit_ZeroCapacity` - Zero capacity
- `test_QueueInit_ZeroElementSize` - Zero element size
- `test_QueueEnqueue_NullQueue` - NULL queue in enqueue
- `test_QueueEnqueue_NullElement` - NULL element in enqueue
- `test_QueueDequeue_NullQueue` - NULL queue in dequeue
- `test_QueueDequeue_NullElement` - NULL element in dequeue
- `test_QueueDequeue_EmptyQueue` - Dequeue from empty queue
- `test_QueueEnqueue_FullQueue` - Enqueue to full queue

### Advanced Features
- `test_QueueFillToCapacity` - Fill queue to maximum capacity
- `test_QueuePeek_Basic` - Basic peek functionality
- `test_QueuePeek_EmptyQueue` - Peek from empty queue
- `test_QueuePeek_NullParameters` - Peek with NULL parameters
- `test_QueuePeekMultiple` - Multiple peek operations
- `test_QueueClear` - Clear queue functionality
- `test_QueueClear_NullQueue` - Clear with NULL queue

### Data Types
- `test_QueueUint32` - Test with 32-bit integers
- `test_QueueStruct` - Test with custom structures
- `test_QueueUtilities_NullQueue` - Utility functions with NULL queue
- `test_QueueMultipleCycles` - Multiple fill/empty cycles

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

The `generate_test_registrations.py` script can automatically scan test files and generate registration code:

```bash
python generate_test_registrations.py
```

This will:
1. Find all test files (`test_*.c`)
2. Extract test function names
3. Generate registration code
4. Output the code to `generated_test_registrations.c`

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

## Best Practices

1. **Test Naming**: Use descriptive names starting with `test_`
2. **Test Organization**: Group related tests in the same file
3. **Error Cases**: Always test error conditions and edge cases
4. **Data Types**: Test with various data types and sizes
5. **Memory Management**: Test with different buffer sizes and configurations

## Coverage

The queue tests provide comprehensive coverage including:
- ✅ Initialization with valid/invalid parameters
- ✅ Basic operations (enqueue/dequeue/peek)
- ✅ Error handling for all failure cases
- ✅ Circular buffer behavior
- ✅ FIFO ordering
- ✅ Multiple data types (uint8_t, uint32_t, structs)
- ✅ Edge cases (empty, full, wrap-around)
- ✅ Multiple operation cycles
- ✅ Utility functions

This ensures the queue implementation is robust and reliable for use throughout the drone project.

---
