#!/usr/bin/env python3

import os
import sys
import subprocess
import urllib.request
from pathlib import Path

FJ_PROJECT_ROOT = str(Path(__file__).parent.parent)
FJ_TOOLS_ROOT_DIR_PATH = os.path.join(FJ_PROJECT_ROOT, "Tools")

MSYS_DIR = os.path.join(FJ_TOOLS_ROOT_DIR_PATH, "msys")
MSYS_EXE_FNAME = "msys2-x86_64-20250830.exe"
MSYS_EXE_URL = f"https://repo.msys2.org/distrib/x86_64/{MSYS_EXE_FNAME}"

MINGW64_GNU_BIN_PATH = os.path.join(MSYS_DIR, "mingw64", "bin")
ARM_GNU_PREFIX = "arm-none-eabi"
ARM_GNU_BIN_PATH = MINGW64_GNU_BIN_PATH


def run_msys_command(command, error_msg):

    try:
        result = subprocess.run(
            [os.path.join(MSYS_DIR, "usr", "bin", "bash"), "-lc", command],
            check=True,
            capture_output=True,
            text=True
        )
        return result
    except subprocess.CalledProcessError as e:
        print(f"ERROR: {error_msg}")
        print(f"Command output: {e.stderr}")
        sys.exit(1)


def install_msys():

    if os.path.isdir(MSYS_DIR):
        print(f"MSYS2 already installed at: {MSYS_DIR}")
        print("Skipping download and installation.")
        return
    
    print("MSYS2 not found. Starting download and installation")
    os.makedirs(FJ_TOOLS_ROOT_DIR_PATH, exist_ok=True)
    
    installer_path = os.path.join(FJ_TOOLS_ROOT_DIR_PATH, MSYS_EXE_FNAME)
    print(f"Downloading MSYS2 from {MSYS_EXE_URL}")
    
    try:
        urllib.request.urlretrieve(MSYS_EXE_URL, installer_path)
    except Exception as e:
        print(f"ERROR: Failed to download MSYS2: {e}")
        sys.exit(1)
    
    print("Installing MSYS2")
    try:
        subprocess.run(
            [installer_path, "in", "--confirm-command", "--accept-messages", "--root", MSYS_DIR],
            check=True
        )
        print("Installation successful, removing msys installer")
        os.remove(installer_path)
        print(f"Installer removed: {installer_path}")
    except subprocess.CalledProcessError:
        print("ERROR: Installation failed, keeping msys installer for retry")
        sys.exit(1)
    
    print("Initializing MSYS2...")
    run_msys_command(':', "MSYS2 initialization failed")
    
    print("Updating MSYS2 core packages...")
    run_msys_command('pacman --noconfirm -Syuu', "MSYS2 core update failed")
    
    print("Installing base development tools...")
    run_msys_command('pacman -S --needed --noconfirm base-devel', "Failed to install dev tools")
    
    print("Installing Mingw toolchain...")
    run_msys_command(
        'pacman -S --needed --noconfirm mingw-w64-x86_64-toolchain',
        "Failed to install mingw toolchain"
    )
    
    print("Installing ARM GCC Toolchain...")
    run_msys_command(
        'pacman -S --needed --noconfirm mingw-w64-x86_64-arm-none-eabi-gcc',
        "Failed to install ARM GCC toolchain"
    )

    print("Installing Clang Toolchain...")
    run_msys_command(
        'pacman -S --needed --noconfirm mingw-w64-x86_64-clang',
        "Failed to install Clang toolchain"
    )

    print("Installing Clang Tidy...")
    run_msys_command(
        'pacman -S --needed --noconfirm mingw-w64-x86_64-clang-tools-extra',
        "Failed to install Clang Tidy"
    )


def verify_tools():

    print("Verifying installed tools...")
    required_tools = {
        "MSYS2 directory": MSYS_DIR,
        "MinGW64 bin directory": MINGW64_GNU_BIN_PATH,
        "mingw32-make": os.path.join(MINGW64_GNU_BIN_PATH, "mingw32-make.exe"),
        "ARM GCC compiler": os.path.join(ARM_GNU_BIN_PATH, f"{ARM_GNU_PREFIX}-gcc.exe"),
        "ARM G++ compiler": os.path.join(ARM_GNU_BIN_PATH, f"{ARM_GNU_PREFIX}-g++.exe"),
        "ARM objcopy": os.path.join(ARM_GNU_BIN_PATH, f"{ARM_GNU_PREFIX}-objcopy.exe"),
        "ARM size": os.path.join(ARM_GNU_BIN_PATH, f"{ARM_GNU_PREFIX}-size.exe"),
    }
    
    all_found = True
    for tool_name, tool_path in required_tools.items():
        if not os.path.exists(tool_path):
            print(f"{tool_name}: NOT FOUND at {tool_path}")
            all_found = False
    
    return all_found


def main():

    print("FlapJack SDK Installation")
    print(f"Project Root: {FJ_PROJECT_ROOT}")
    
    install_msys()
    
    if verify_tools():
        print("SDK installation completed successfully!")
    else:
        print("SDK installation completed with errors!")
        print("Some required tools are missing.")
        sys.exit(1)


if __name__ == "__main__":
    main()