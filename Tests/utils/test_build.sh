#!/bin/bash
# Test script for CMake + CMock setup

echo "=== Testing CMake + CMock Setup ==="

# Configure the build
echo "Configuring build..."
cmake -B build -S .

if [ $? -ne 0 ]; then
    echo "❌ CMake configuration failed!"
    exit 1
fi

echo "✅ CMake configuration successful!"

# Build the project
echo "Building project..."
cmake --build build

if [ $? -ne 0 ]; then
    echo "❌ Build failed!"
    exit 1
fi

echo "✅ Build successful!"

# Run tests
echo "Running tests..."
cd build && ctest --verbose

if [ $? -ne 0 ]; then
    echo "❌ Tests failed!"
    exit 1
fi

echo "✅ All tests passed!"
echo "=== CMake + CMock setup complete! ==="