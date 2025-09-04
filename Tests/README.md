## Quick Start

From the `Tests/` directory, run:

    ./test.bash

## Test Structure

All test functions should follow this naming convention:
```c
void test_FunctionName(void) {
    // Test implementation
    TEST_ASSERT_EQUAL(expected, actual);
}
```
