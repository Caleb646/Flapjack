#!/usr/bin/env python3

import os
import sys
import subprocess
import argparse
from pathlib import Path

PLATFORM = sys.platform

FJ_PROJECT_ROOT = str(Path(__file__).parent.parent)
FJ_BUILD_ROOT_DIR_PATH = os.path.join(FJ_PROJECT_ROOT, "Build")
FJ_TOOLS_ROOT_DIR_PATH = os.path.join(FJ_PROJECT_ROOT, "Tools")

if PLATFORM == "win32":
    MSYS_DIR = os.path.join(FJ_TOOLS_ROOT_DIR_PATH, "msys")
    MINGW64_GNU_BIN_PATH = os.path.join(MSYS_DIR, "mingw64", "bin")
    ARM_GNU_BIN_PATH = MINGW64_GNU_BIN_PATH
    CMAKE_GENERATOR = "MinGW Makefiles"
else:
    ARM_GNU_BIN_PATH = os.path.join(FJ_TOOLS_ROOT_DIR_PATH, "arm-gnu", "bin")
    CMAKE_GENERATOR = "Unix Makefiles"

VALID_BOARDS = ["nucleo-h747zi", "flapjack-v1"]


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="FlapJack Firmware Build Script",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"""
Examples:
  fw_build.py {VALID_BOARDS[0]}
  fw_build.py {VALID_BOARDS[0]} --build-type Release
  fw_build.py --board {VALID_BOARDS[0]} --build-type Debug

Valid boards: {', '.join(VALID_BOARDS)}
        """
    )
    parser.add_argument(
        "board",
        nargs="?",
        choices=VALID_BOARDS,
        help=f"Board configuration: {', '.join(VALID_BOARDS)}"
    )
    parser.add_argument(
        "-b", "--board",
        dest="board_kwarg",
        choices=VALID_BOARDS,
        help="Board configuration (alternative to positional argument)"
    ) 
    parser.add_argument(
        "-t", "--build-type",
        default="Debug",
        choices=["Debug", "Release"],
        help="Build type: Debug or Release (default: Debug)"
    )
    args = parser.parse_args()
    board_name = args.board_kwarg if args.board_kwarg else args.board
    if not board_name:
        parser.error("Board configuration is required (either as positional or --board argument)")
    
    print(f"Using {board_name} configuration")
    print(f"Using {args.build_type} build configuration")
    
    return board_name, args.build_type


def verify_toolchain():
    if PLATFORM == "win32":
        required_paths = [
            MSYS_DIR,
            MINGW64_GNU_BIN_PATH,
            os.path.join(MINGW64_GNU_BIN_PATH, "mingw32-make.exe"),
            os.path.join(ARM_GNU_BIN_PATH, "arm-none-eabi-gcc.exe"),
        ]
    else:
        required_paths = [
            ARM_GNU_BIN_PATH,
            os.path.join(ARM_GNU_BIN_PATH, "arm-none-eabi-gcc"),
        ]

    missing_paths = [p for p in required_paths if not os.path.exists(p)]

    if missing_paths:
        print("ERROR: Required toolchain paths are missing:")
        for path in missing_paths:
            print(f"  - {path}")
        print("Please run sdk_install.py first to install the required tools.")
        sys.exit(1)


def run_cmake_build(board_name, build_type):
    env = os.environ.copy()
    if PLATFORM == "win32":
        env["PATH"] = f"{MINGW64_GNU_BIN_PATH}{os.pathsep}{ARM_GNU_BIN_PATH}{os.pathsep}{env['PATH']}"
    else:
        env["PATH"] = f"{ARM_GNU_BIN_PATH}{os.pathsep}{env['PATH']}"
    output_dir = os.path.join(FJ_BUILD_ROOT_DIR_PATH, board_name, build_type)
    cmake_args = [
        "cmake",
        # "--fresh",
        "-S", FJ_PROJECT_ROOT,
        "-B", output_dir,
        "-G", CMAKE_GENERATOR,
        f"-DCMAKE_BUILD_TYPE={build_type}",
        f"-DBOARD_NAME={board_name}",
    ]

    cmake_cfg_log_path = os.path.join(output_dir, "cmake_configure.log")
    cmake_build_log_path = os.path.join(output_dir, "cmake_build.log")
    os.makedirs(os.path.dirname(cmake_cfg_log_path), exist_ok=True)
    os.makedirs(os.path.dirname(cmake_build_log_path), exist_ok=True)
    if not os.path.isfile(cmake_cfg_log_path):
        open(cmake_cfg_log_path, "x").close()
    if not os.path.isfile(cmake_build_log_path):
        open(cmake_build_log_path, "x").close()
    
    print(f"CMAKE ARGS: {' '.join(cmake_args)}")
    print("Configuring project...")
    try:
        subprocess.run(cmake_args, 
                        stdout=open(cmake_cfg_log_path, "w"), 
                        stderr=subprocess.STDOUT,
                        check=True, 
                        env=env
                       )
    except subprocess.CalledProcessError:
        print(f"ERROR: CMake configuration failed. Check {cmake_cfg_log_path} for details")
        sys.exit(1)

    print("Building project...")
    try:
        subprocess.run(["cmake", "--build", output_dir, "-j"], 
                       stdout=open(cmake_build_log_path, "w"), 
                       stderr=subprocess.STDOUT,
                       check=True, 
                       env=env
                       )
    except subprocess.CalledProcessError:
        print(f"ERROR: CMake build failed. Check {cmake_build_log_path} for details")
        sys.exit(1)
    
    print("Build completed successfully!")
    print(f"Build artifacts are in: {output_dir}")


def main():
    print("FlapJack Firmware Build")
    print(f"Project Root: {FJ_PROJECT_ROOT}")
    
    board_name, build_type = parse_arguments()
    
    verify_toolchain()
    run_cmake_build(board_name, build_type)

if __name__ == "__main__":
    main()