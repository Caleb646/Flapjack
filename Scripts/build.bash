#!/bin/bash

set -e          # Exit immediately if a command exits with a non-zero status
set -u          # Treat unset variables as an error
set -o pipefail # Make pipelines fail if any command in the pipeline fails
# set -x        # Print commands and their arguments as they are executed

source ./Scripts/config.bash

mkdir -p -v "${FJ_TOOLS_ROOT_DIR_PATH}"
mkdir -p -v "${FJ_BUILD_ROOT_DIR_PATH}"

BOARD_CONF=$1
BUILD_TYPE=${2:-Debug}
BUILD_MODE=${3:-Firmware}

assert_paths_exist() {

    local path
    if [ $# -eq 0 ]; then
        echo "ERROR: No paths provided to check"
        exit 1
    fi
    
    for path in "$@"; do
        if [ ! -e "${path}" ]; then
            echo "ERROR Path missing: ${path}"
            exit 1
        fi
    done

    return 0
}

install_msys() {

    if [ -d "${MSYS_DIR}" ]; then
        echo "MSYS2 already installed at: ${MSYS_DIR}"
        echo "Skipping download and installation."
        return 0
    fi

    echo "MSYS2 not found. Starting download and installation"
    echo "Downloading MSYS2"
    curl -L -o "${FJ_TOOLS_ROOT_DIR_PATH}/${MSYS_EXE_FNAME}" "${MSYS_EXE_URL}"
    assert_paths_exist "${FJ_TOOLS_ROOT_DIR_PATH}/${MSYS_EXE_FNAME}"

    echo "Installing MSYS2"
    ${FJ_TOOLS_ROOT_DIR_PATH}/${MSYS_EXE_FNAME} in --confirm-command --accept-messages --root ${MSYS_DIR}

    if [ $? -eq 0 ]; then
        echo "Installation successful, removing msys installer"
        rm "${FJ_TOOLS_ROOT_DIR_PATH}/${MSYS_EXE_FNAME}"
        echo "Installer removed: ${FJ_TOOLS_ROOT_DIR_PATH}/${MSYS_EXE_FNAME}"
        return 0
    else
        echo "Installation failed, keeping msys installer for retry"
        exit 1
    fi

    # Run for the first time
    ${MSYS_DIR}/usr/bin/bash -lc ':' || { echo "ERROR: MSYS2 initialization failed"; exit 1; }
    # Update MSYS2 -- Core update (in case any core packages are outdated)
    ${MSYS_DIR}/usr/bin/bash -lc 'pacman --noconfirm -Syuu' || { echo "ERROR: MSYS2 core update failed"; exit 1; }
    # Install base development tools
    ${MSYS_DIR}/usr/bin/bash -lc 'pacman -S --needed --noconfirm base-devel' || { echo "ERROR: MSYS2 failed to install dev tools"; exit 1; }
    # Install Mingw toolchain for Mingw Make for CMake to be able to generator Makefiles and
    # gcc to build and run tests on host
    # ${MSYS_DIR}/usr/bin/bash -lc 'pacman -S --needed --noconfirm mingw-w64-x86_64-make'
    ${MSYS_DIR}/usr/bin/bash -lc 'pacman -S --needed --noconfirm mingw-w64-x86_64-toolchain' || { echo "ERROR: MSYS2 failed to install mingw toolchain"; exit 1; }
    # Install ARM GCC Toolchain
    ${MSYS_DIR}/usr/bin/bash -lc 'pacman -S --needed --noconfirm mingw-w64-x86_64-arm-none-eabi-gcc' || { echo "ERROR: MSYS2 failed to install ARM GCC toolchain"; exit 1; }
}

install_msys
assert_paths_exist "${MSYS_DIR}" "${MINGW64_GNU_BIN_PATH}" "${MINGW64_GNU_BIN_PATH}/mingw32-make.exe" "${ARM_GNU_BIN_PATH}"

BOARD_CONF=$1
case "${BOARD_CONF}" in
    "DEV_BOARD"|"DEVELOPMENT"|"dev")
        BOARD_CONF="DEV_BOARD"
        echo "Using DEV_BOARD configuration"
        ;;
    "MY_BOARD"|"CUSTOM"|"custom")
        BOARD_CONF="MY_BOARD"
        echo "Using MY_BOARD configuration"
        ;;
    "PROD_BOARD"|"PRODUCTION"|"prod")
        BOARD_CONF="PROD_BOARD"
        echo "Using PROD_BOARD configuration"
        ;;
    *)
        echo "ERROR: Invalid board configuration '${BOARD_CONF}'"
        echo "Valid options are:"
        echo "  DEV_BOARD (or dev, development)"
        echo "  MY_BOARD  (or custom)"
        echo "  PROD_BOARD (or prod, production)"
        exit 1
        ;;
esac

BUILD_TYPE=${2:-Debug}  # Default to Debug
case "${BUILD_TYPE}" in
    "Debug"|"debug"|"DEBUG")
        BUILD_TYPE="Debug"
        echo "Using Debug build configuration"
        ;;
    "Release"|"release"|"RELEASE")
        BUILD_TYPE="Release"
        echo "Using Release build configuration"
        ;;
    *)
        echo "ERROR: Invalid build type '${BUILD_TYPE}'"
        echo "Valid options are: Debug, Release"
        exit 1
        ;;
esac

export PATH="${MINGW64_GNU_BIN_PATH}:${ARM_GNU_BIN_PATH}:${PATH}"
OUTPUT_DIR="${FJ_BUILD_ROOT_DIR_PATH}/${BUILD_TYPE}"
cmake_args=(
    # --fresh
    -S "${FJ_PROJECT_ROOT}"
    -B "${OUTPUT_DIR}" 
    -G "${CMAKE_GENERATOR}"
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
    -DCMAKE_TOOLCHAIN_FILE="${CMAKE_TOOLCHAIN_FILE}"
    -DBOARD_CONFIG="${BOARD_CONF}"
)

echo "CMAKE ARGS: ${cmake_args[@]}"
sleep 1

cmake "${cmake_args[@]}"
cmake --build "${OUTPUT_DIR}" # --verbose

exit 0
# echo "Checking toolchain file..."
# if [ ! -f "./cmake/gcc_tc.cmake" ]; then
#     echo "ERROR: Toolchain file not found!"
#     exit 1
# fi

# cmake -S . -B Build -G "MinGW Makefiles" -DCMAKE_TOOLCHAIN_FILE=./cmake/gcc_tc.cmake -DBUILD_TESTS=ON
# cmake --build Build # --verbose 2>&1 | tee ./Build/build_log.txt

# Append mingw to the front of the path to ensure the correct gcc/g++ are used
# export PATH="/c/msys64/ucrt64/bin:$PATH"

# cmake -S . -B Build/tests \
#     -DCMAKE_C_COMPILER=gcc \
#     -DCMAKE_CXX_COMPILER=g++ \
#     -G "MinGW Makefiles" \
#     -DBUILD_TESTS=ON \
#     -DCMAKE_BUILD_TYPE=Debug

# cmake --build Build/tests


# # ctest -V --test-dir Build/tests -C Debug --output-on-failure
# ctest --test-dir Build/tests -C Debug --output-on-failure