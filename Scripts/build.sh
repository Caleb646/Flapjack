#!/bin/bash

install_msys() {

    echo "MSYS2 not found. Starting download and installation..."
    if ! mkdir -p "${TOOLS_DIR}"; then
        echo "ERROR: Failed to create directory ${TOOLS_DIR}"
        return 1
    fi

    echo "Downloading MSYS2"
    curl -L -o "${TOOLS_DIR}/${MSYS_EXE_FNAME}" "${MSYS_EXE_URL}"
    
    if [ ! -f "${TOOLS_DIR}/${MSYS_EXE_FNAME}" ]; then
        echo "ERROR: Download failed"
        return 1
    fi

    echo "Installing MSYS2"
    ${TOOLS_DIR}/${MSYS_EXE_FNAME} in --confirm-command --accept-messages --root ${MSYS_DIR}

    if [ $? -eq 0 ]; then
        echo "Installation successful, removing msys installer..."
        rm "${TOOLS_DIR}/${MSYS_EXE_FNAME}"
        echo "Installer removed: ${TOOLS_DIR}/${MSYS_EXE_FNAME}"
        return 0
    else
        echo "Installation failed, keeping msys installer for retry"
        return 1
    fi
}

if [ ! -d "${TOOLS_DIR}" ]; then
    echo "TOOLS_DIR ${$TOOLS_DIR} does not exist"
    exit 1
fi

if [ -d "${MSYS_DIR}" ]; then
    echo "MSYS2 already installed at: ${MSYS_DIR}"
    echo "Skipping download and installation."
else

    install_msys
    
    if [ $? -eq 0 ]; then
        echo "MSYS2 successfully installed at: ${MSYS_DIR}"
    else
        echo "MSYS2 installation failed"
        exit 1
    fi

    # Run for the first time
    ${MSYS_DIR}/usr/bin/bash -lc ':'
    # Update MSYS2 -- Core update (in case any core packages are outdated)
    ${MSYS_DIR}/usr/bin/bash -lc 'pacman --noconfirm -Syuu'
    # Install base development tools
    ${MSYS_DIR}/usr/bin/bash -lc 'pacman -S --needed --noconfirm base-devel'
    # Install Mingw toolchain for Mingw Make for CMake to be able to generator Makefiles and
    # gcc to build and run tests on host
    # ${MSYS_DIR}/usr/bin/bash -lc 'pacman -S --needed --noconfirm mingw-w64-x86_64-make'
    ${MSYS_DIR}/usr/bin/bash -lc 'pacman -S --needed --noconfirm mingw-w64-x86_64-toolchain'
    # Install ARM GCC Toolchain
    ${MSYS_DIR}/usr/bin/bash -lc 'pacman -S --needed --noconfirm mingw-w64-x86_64-arm-none-eabi-gcc'

fi

if [ ! -d "${MINGW64_GNU_BIN_PATH}" ]; then
    echo "ERROR: ARM GNU Toolchain path not found at ${MINGW64_GNU_BIN_PATH}"
    exit 1
fi

if [ ! -f "${MINGW64_GNU_BIN_PATH}/mingw32-make.exe" ]; then
    echo "ERROR: mingw32-make.exe not found in ${MINGW64_GNU_BIN_PATH}"
    exit 1
fi

if [ ! -d "${STM32_TOOLCHAIN_PATH}" ]; then
    echo "${STM32_TOOLCHAIN_PATH} does not exist."
    exit 1
fi

if [ $# -eq 0 ]; then
    echo "Usage: $0 <BOARD_CONFIG>"
    echo "Available board configurations:"
    echo "  DEV_BOARD - Development board configuration"
    echo "  MY_BOARD  - Custom board configuration"
    exit 1
fi

# cmake --fresh -S . -B Build/stm32 -G "MinGW Makefiles"
cmake -S ${PROJECT_ROOT} -B ${STM32_BUILD_DIR} -G ${CMAKE_GENERATOR} -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE} -DBOARD_CONFIG=${BOARD_CONF} # -DCMAKE_BUILD_TYPE=Debug
cmake --build ${STM32_BUILD_DIR} # --verbose



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