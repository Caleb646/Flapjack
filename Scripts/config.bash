#!/bin/bash

export FJ_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export FJ_SCRIPTS_ROOT_DIR_PATH="${FJ_PROJECT_ROOT}/Scripts"
export FJ_TOOLS_ROOT_DIR_PATH="${FJ_PROJECT_ROOT}/Tools"
export FJ_BUILD_ROOT_DIR_PATH="${FJ_PROJECT_ROOT}/Build"
# export STM32_BUILD_DIR="${FJ_BUILD_ROOT_DIR_PATH}/stm32"
# export STM32_CM7_ELF_PATH="${STM32_BUILD_DIR}/m7core.elf"
# export STM32_CM4_ELF_PATH="${STM32_BUILD_DIR}/m4core.elf"
export MSYS_DIR="${FJ_TOOLS_ROOT_DIR_PATH}/msys"
export MSYS_EXE_FNAME="msys2-x86_64-20250830.exe"
export MSYS_EXE_URL="https://repo.msys2.org/distrib/x86_64/${MSYS_EXE_FNAME}"
export MINGW64_GNU_BIN_PATH="${MSYS_DIR}/mingw64/bin"
export ARM_GNU_PREFIX="arm-none-eabi"
export ARM_GNU_BIN_PATH=${MINGW64_GNU_BIN_PATH}
export CMAKE_TOOLCHAIN_FILE="${FJ_PROJECT_ROOT}/cmake/gcc_tc.cmake"
export CMAKE_GENERATOR="MinGW Makefiles"
# update path to include mingw gnu and arm gnu
# export PATH="${MINGW64_GNU_BIN_PATH}:${ARM_GNU_BIN_PATH}:${PATH}"
export TESTS_TARGET_NAME="Tests"
# stm32-cmake env setup
export STM32_CM7_TARGET_NAME="m7core"
export STM32_CM4_TARGET_NAME="m4core"
# stm32-cmake appends bin to the path so remove bin from the arm gnu path
export STM32_TOOLCHAIN_PATH=$(dirname "$ARM_GNU_BIN_PATH") #"${MSYS_DIR}/mingw64"
export STM32_TARGET_TRIPLET=${ARM_GNU_PREFIX}
export STM32_CUBE_H7_PATH="${FJ_PROJECT_ROOT}/Vendor/STM32CubeH7"
export FREERTOS_PATH="${STM32_CUBE_H7_PATH}/Middlewares/Third_Party"
echo "Project Root: $FJ_PROJECT_ROOT"
# exit 0