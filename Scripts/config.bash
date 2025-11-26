#!/bin/bash

export PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export SCRIPTS_DIR="${PROJECT_ROOT}/Scripts"
export TOOLS_DIR="${PROJECT_ROOT}/Tools"
export BUILD_DIR="${PROJECT_ROOT}/Build"

mkdir -p -v "${TOOLS_DIR}"
mkdir -p -v "${BUILD_DIR}"

export STM32_BUILD_DIR="${BUILD_DIR}/stm32"
export STM32_CM7_ELF_PATH="${STM32_BUILD_DIR}/m7core.elf"
export STM32_CM4_ELF_PATH="${STM32_BUILD_DIR}/m4core.elf"

export TESTS_BUILD_DIR="${BUILD_DIR}/tests"

export MSYS_DIR="${TOOLS_DIR}/msys"
export MSYS_EXE_FNAME="msys2-x86_64-20250830.exe"
export MSYS_EXE_URL="https://repo.msys2.org/distrib/x86_64/${MSYS_EXE_FNAME}"

export MINGW64_GNU_BIN_PATH="${MSYS_DIR}/mingw64/bin"

export ARM_GNU_PREFIX="arm-none-eabi"
export ARM_GNU_BIN_PATH=${MINGW64_GNU_BIN_PATH}

export CMAKE_TOOLCHAIN_FILE="${PROJECT_ROOT}/cmake/stm32-cmake/cmake/stm32_gcc.cmake"
export CMAKE_GENERATOR="MinGW Makefiles"

# update path to include mingw gnu and arm gnu
export PATH="${MINGW64_GNU_BIN_PATH}:${ARM_GNU_BIN_PATH}:${PATH}"

# stm32-cmake env setup
# stm32-cmake appends bin to the path so remove bin from the arm gnu path
export STM32_TOOLCHAIN_PATH=$(dirname "$ARM_GNU_BIN_PATH") #"${MSYS_DIR}/mingw64"
export STM32_TARGET_TRIPLET=${ARM_GNU_PREFIX}
export STM32_CUBE_H7_PATH="${PROJECT_ROOT}/Vendor/STM32CubeH7"
export FREERTOS_PATH="${STM32_CUBE_H7_PATH}/Middlewares/Third_Party"

echo "Project Root: $PROJECT_ROOT"