#!/usr/bin/env python3
"""
Flapjack board management CLI.

Commands
--------
  install           Install SDK tools (ARM GCC, LLVM, OpenOCD)
  build             Configure and build firmware
  flash             Flash firmware; optionally run HIL tests

Examples
--------
  python3 Scripts/board.py install
  python3 Scripts/board.py build --board nucleo-h747zi
  python3 Scripts/board.py build --board nucleo-h747zi -f r
  python3 Scripts/board.py build --board nucleo-h747zi -f dc
  python3 Scripts/board.py build --board nucleo-h747zi -f rc
  python3 Scripts/board.py build --board nucleo-h747zi -f gcr
  python3 Scripts/board.py flash --board nucleo-h747zi
  python3 Scripts/board.py flash --board nucleo-h747zi -f r
  python3 Scripts/board.py flash --board nucleo-h747zi -f t
  python3 Scripts/board.py flash --board nucleo-h747zi -f dt --port /dev/ttyUSB0 --baud 115200
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
import tarfile
import time
import urllib.request
from pathlib import Path

# ── Project paths ──────────────────────────────────────────────────────────────

PLATFORM = sys.platform

PROJECT_ROOT  = Path(__file__).parent.parent
BUILD_ROOT    = PROJECT_ROOT / "Build"
TOOLS_ROOT    = PROJECT_ROOT / "Tools"
GUI_ROOT      = PROJECT_ROOT / "GUI"

BOARDS  = ["nucleo-h747zi", "flapjack-v1"]
CONFIGS = ["Debug", "Release"]

# ── Generation ─────────────────────────────────────────────────────

UMSG_FILES_DIR          = PROJECT_ROOT / "Firmware" / "msgs" / "umsg" / "defs"
UMSG_OUTPUT_DIR         = PROJECT_ROOT / "Firmware" / "msgs" / "umsg"

PROTO_FILES_DIR         = PROJECT_ROOT / "Firmware" / "msgs" / "proto" / "defs"
PROTO_OUTPUT_DIR        = PROJECT_ROOT / "Firmware" / "msgs" / "proto"

PROTOC_EXE       = TOOLS_ROOT / "protoc" / "bin" / "protoc"
if PLATFORM == "win32":
    PROTOC_EXE   = PROTOC_EXE.with_suffix(".exe")
NANOPB_ROOT      = PROJECT_ROOT / "Vendor" / "nanopb"
NANOPB_GEN       = PROJECT_ROOT / "Vendor" / "nanopb" / "generator" / "nanopb_generator.py"
UMSG_GEN         = PROJECT_ROOT / "Vendor" / "umsg" / "umsg_gen" / "umsg_gen" / "umsg_gen.py"
NANOPB_PROTO_PY  = NANOPB_ROOT / "generator" / "proto"

VENV_PYTHON             = PROJECT_ROOT / ".venv" / "bin" / "python3"
if PLATFORM == "win32":
    VENV_PYTHON         = PROJECT_ROOT / ".venv" / "Scripts" / "python.exe"

# ── Toolchain paths ────────────────────────────────────────────────────────────

# Windows / MSYS2
MSYS_DIR            = TOOLS_ROOT / "msys"
MSYS_EXE_FNAME      = "msys2-x86_64-20250830.exe"
MSYS_EXE_URL        = f"https://repo.msys2.org/distrib/x86_64/{MSYS_EXE_FNAME}"
MINGW64_BIN         = MSYS_DIR / "mingw64" / "bin"

# Linux — ARM GNU (xPack)
ARM_GNU_VERSION = "15.2.1-1.1"
ARM_GNU_TARBALL = f"xpack-arm-none-eabi-gcc-{ARM_GNU_VERSION}-linux-x64.tar.gz"
ARM_GNU_URL     = (
    f"https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download"
    f"/v{ARM_GNU_VERSION}/{ARM_GNU_TARBALL}"
)
ARM_GNU_DIR = TOOLS_ROOT / "arm-gnu"
ARM_GNU_BIN = ARM_GNU_DIR / "bin"

# Linux — LLVM/Clang
LLVM_VERSION = "22.1.5"
LLVM_DIRNAME = f"LLVM-{LLVM_VERSION}-Linux-X64"
LLVM_TARBALL = f"{LLVM_DIRNAME}.tar.xz"
LLVM_URL     = (
    f"https://github.com/llvm/llvm-project/releases/download"
    f"/llvmorg-{LLVM_VERSION}/{LLVM_TARBALL}"
)
LLVM_DIR = TOOLS_ROOT / "llvm"
LLVM_BIN = LLVM_DIR / "bin"

# Linux — OpenOCD (xPack)
OPENOCD_VERSION = "0.12.0-7"
OPENOCD_DIRNAME = f"xpack-openocd-{OPENOCD_VERSION}"
OPENOCD_TARBALL = f"{OPENOCD_DIRNAME}-linux-x64.tar.gz"
OPENOCD_URL     = (
    f"https://github.com/xpack-dev-tools/openocd-xpack/releases/download"
    f"/v{OPENOCD_VERSION}/{OPENOCD_TARBALL}"
)
OPENOCD_DIR = TOOLS_ROOT / "openocd"
OPENOCD_BIN = OPENOCD_DIR / "bin"

_HOST_ARM_BIN    = ARM_GNU_BIN
_OPENOCD_PATH    = OPENOCD_BIN / "openocd"
CMAKE_GENERATOR = "Unix Makefiles"
if PLATFORM == "win32":
    _HOST_ARM_BIN    = MINGW64_BIN
    _OPENOCD_PATH    = MINGW64_BIN / "openocd.exe"
    _CMAKE_GENERATOR = "MinGW Makefiles"

# ── OpenOCD flash constants ────────────────────────────────────────────────────

_IFACE_CFG     = "interface/stlink.cfg"
_TARGET_DUAL   = "Scripts/stm32h747_dual_core.cfg"


# ══════════════════════════════════════════════════════════════════════════════
#  install
# ══════════════════════════════════════════════════════════════════════════════

def _run_msys(command: str, error_msg: str) -> None:
    bash = MSYS_DIR / "usr" / "bin" / "bash"
    try:
        subprocess.run([str(bash), "-lc", command], check=True,
                       capture_output=True, text=True)
    except subprocess.CalledProcessError as e:
        print(f"ERROR: {error_msg}\n{e.stderr}")
        sys.exit(1)


def _install_msys() -> None:
    if MSYS_DIR.is_dir():
        print(f"MSYS2 already installed at: {MSYS_DIR}")
        return

    print("MSYS2 not found. Downloading and installing …")
    TOOLS_ROOT.mkdir(parents=True, exist_ok=True)
    installer = TOOLS_ROOT / MSYS_EXE_FNAME

    try:
        urllib.request.urlretrieve(MSYS_EXE_URL, installer)
    except Exception as e:
        print(f"ERROR: Failed to download MSYS2: {e}")
        sys.exit(1)

    try:
        subprocess.run([str(installer), "in", "--confirm-command",
                        "--accept-messages", "--root", str(MSYS_DIR)], check=True)
        installer.unlink()
    except subprocess.CalledProcessError:
        print("ERROR: MSYS2 installation failed (installer kept for retry)")
        sys.exit(1)

    _run_msys(":", "MSYS2 initialization failed")
    _run_msys("pacman --noconfirm -Syuu", "MSYS2 core update failed")
    _run_msys("pacman -S --needed --noconfirm base-devel", "Failed to install dev tools")
    _run_msys("pacman -S --needed --noconfirm mingw-w64-x86_64-toolchain", "Failed to install MinGW toolchain")
    _run_msys("pacman -S --needed --noconfirm mingw-w64-x86_64-arm-none-eabi-gcc", "Failed to install ARM GCC")
    _run_msys("pacman -S --needed --noconfirm mingw-w64-x86_64-clang", "Failed to install Clang")
    _run_msys("pacman -S --needed --noconfirm mingw-w64-x86_64-clang-tools-extra", "Failed to install Clang-tidy")
    _run_msys("pacman -S --needed --noconfirm mingw-w64-x86_64-openocd", "Failed to install OpenOCD")
    print("MSYS2 installation complete.")


def _download_progress(block_num: int, block_size: int, total_size: int) -> None:
    if total_size > 0:
        pct = min(100, block_num * block_size * 100 // total_size)
        print(f"\r  {pct}%", end="", flush=True)


def _download_and_extract(url: str, tarball: str, dest: Path, label: str) -> None:
    if dest.is_dir():
        print(f"{label} already installed at: {dest}")
        return

    TOOLS_ROOT.mkdir(parents=True, exist_ok=True)
    tarball_path = TOOLS_ROOT / tarball

    print(f"Downloading {label} …\n  {url}")
    try:
        urllib.request.urlretrieve(url, tarball_path, reporthook=_download_progress)
        print()
    except Exception as e:
        print(f"\nERROR: Failed to download {label}: {e}")
        sys.exit(1)

    print(f"Extracting {label} …")
    try:
        with tarfile.open(tarball_path) as tf:
            top_level = tf.getnames()[0].split("/")[0]
            tf.extractall(TOOLS_ROOT)
    except Exception as e:
        print(f"ERROR: Failed to extract {label}: {e}")
        sys.exit(1)

    extracted = TOOLS_ROOT / top_level
    if extracted != dest:
        extracted.rename(dest)
    tarball_path.unlink()
    print(f"Done: {label} installed at {dest}")


def _install_linux() -> None:
    _download_and_extract(ARM_GNU_URL, ARM_GNU_TARBALL, ARM_GNU_DIR, "ARM GNU Toolchain")
    _download_and_extract(LLVM_URL,    LLVM_TARBALL,    LLVM_DIR,    "LLVM/Clang")
    _download_and_extract(OPENOCD_URL, OPENOCD_TARBALL, OPENOCD_DIR, "OpenOCD")


def _check_paths(tools: dict[str, Path]) -> bool:
    ok = True
    for name, path in tools.items():
        if path.exists():
            print(f"  OK      {name}")
        else:
            print(f"  MISSING {name}  ({path})")
            ok = False
    return ok


def _verify_install() -> bool:
    print("Verifying installed tools …")
    pfx = "arm-none-eabi"
    if PLATFORM == "win32":
        return _check_paths({
            "MSYS2":          MSYS_DIR,
            "MinGW64 bin":    MINGW64_BIN,
            "mingw32-make":   MINGW64_BIN / "mingw32-make.exe",
            "ARM GCC":        MINGW64_BIN / f"{pfx}-gcc.exe",
            "ARM G++":        MINGW64_BIN / f"{pfx}-g++.exe",
            "ARM objcopy":    MINGW64_BIN / f"{pfx}-objcopy.exe",
            "ARM size":       MINGW64_BIN / f"{pfx}-size.exe",
        })
    return _check_paths({
        "ARM GNU toolchain":     ARM_GNU_DIR,
        "arm-none-eabi-gcc":     ARM_GNU_BIN / f"{pfx}-gcc",
        "arm-none-eabi-g++":     ARM_GNU_BIN / f"{pfx}-g++",
        "arm-none-eabi-gdb":     ARM_GNU_BIN / f"{pfx}-gdb",
        "arm-none-eabi-objcopy": ARM_GNU_BIN / f"{pfx}-objcopy",
        "arm-none-eabi-size":    ARM_GNU_BIN / f"{pfx}-size",
        "LLVM/Clang":            LLVM_DIR,
        "clang":                 LLVM_BIN / "clang",
        "clang++":               LLVM_BIN / "clang++",
        "clang-tidy":            LLVM_BIN / "clang-tidy",
        "openocd":               OPENOCD_BIN / "openocd",
    })


def cmd_install(_: argparse.Namespace) -> None:
    print(f"Flapjack SDK Install  |  platform: {PLATFORM}  |  root: {PROJECT_ROOT}\n")
    if PLATFORM == "win32":
        _install_msys()
    elif PLATFORM.startswith("linux"):
        _install_linux()
    else:
        print(f"ERROR: Unsupported platform '{PLATFORM}'")
        sys.exit(1)

    if _verify_install():
        print("\nSDK installation complete.")
    else:
        print("\nSDK installation finished with errors — some tools are missing.")
        sys.exit(1)


# ══════════════════════════════════════════════════════════════════════════════
#  build
# ══════════════════════════════════════════════════════════════════════════════

def _generate_proto() -> None:
    print("Generating protobuf sources …")

    proto_files = sorted(PROTO_FILES_DIR.glob("*.proto"))
    if not proto_files:
        print(f"WARNING: No .proto files found in {PROTO_FILES_DIR}")
        return

    if not NANOPB_GEN.exists():
        print(f"ERROR: nanopb_generator not found at {NANOPB_GEN}")
        sys.exit(1)

    PROTO_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    for proto_file in proto_files:
        try:
            subprocess.run(
                [str(VENV_PYTHON), str(NANOPB_GEN), str(proto_file),
                "-D", str(PROTO_OUTPUT_DIR), "-I", str(PROTO_FILES_DIR)],
                check=True, capture_output=True, text=True, env=os.environ.copy(),
            )
            subprocess.run(
                [str(PROTOC_EXE), str(proto_file),
                 f"-I{PROTO_FILES_DIR}", f"--python_out={GUI_ROOT}"],
                check=True, capture_output=True, shell=True, text=True, env=os.environ.copy(),
            )
        except subprocess.CalledProcessError as e:
            print(f"ERROR: protoc failed on {proto_file.name}: {e.stderr}")
            raise e


def _generate_umsg() -> None:
    print("Generating umsg sources …")

    if not UMSG_GEN.exists():
        print(f"ERROR: umsg_gen not found at {UMSG_GEN}")
        sys.exit(1)

    UMSG_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    try:
        subprocess.run(
            [str(VENV_PYTHON), str(UMSG_GEN),
             "-d", str(UMSG_FILES_DIR), "-o", str(UMSG_OUTPUT_DIR)],
            check=True, capture_output=True, text=True, env=os.environ.copy(),
        )
    except subprocess.CalledProcessError as e:
        print(f"ERROR: umsg_gen failed: {e.stderr}")
        sys.exit(1)


def _verify_toolchain() -> None:
    if PLATFORM == "win32":
        missing = [str(MINGW64_BIN), str(MINGW64_BIN / "mingw32-make.exe"),
                   str(MINGW64_BIN / "arm-none-eabi-gcc.exe")]
    else:
        missing = [str(ARM_GNU_BIN), str(ARM_GNU_BIN / "arm-none-eabi-gcc")]

    missing = [p for p in missing if not Path(p).exists()]
    if missing:
        print("ERROR: Toolchain missing — run `board.py install` first:")
        for p in missing:
            print(f"  {p}")
        sys.exit(1)


def cmd_build(args: argparse.Namespace) -> None:
    # ── Parse flags string character-by-character ──────────────────────────────
    config      = 'Debug'
    clean       = False
    build_tests = False
    regen       = False

    for ch in args.flags.lower():
        if   ch == 'd': config      = 'Debug'
        elif ch == 'r': config      = 'Release'
        elif ch == 'c': clean       = True
        elif ch == 't': build_tests = True
        elif ch == 'g': regen       = True
        else:
            print(f"ERROR: Unknown flag '{ch}'. Valid: d=Debug, r=Release, c=clean, t=tests, g=generate")
            sys.exit(1)

    print(f"Flapjack Firmware Build  |  board: {args.board}  |  config: {config}  |  drivers: {args.drivers}\n")
    _verify_toolchain()

    if clean or regen:
        _generate_proto()
        _generate_umsg()

    env = os.environ.copy()
    env["PATH"] = str(_HOST_ARM_BIN) + os.pathsep + env["PATH"]

    output_dir = BUILD_ROOT / args.board / config

    if clean and output_dir.exists():
        print(f"Cleaning {output_dir} …")
        shutil.rmtree(output_dir)

    cmake_cfg_log   = output_dir / "cmake_configure.log"
    cmake_build_log = output_dir / "cmake_build.log"
    os.makedirs(output_dir, exist_ok=True)

    cmake_args = [
        "cmake",
        "-S", str(PROJECT_ROOT),
        "-B", str(output_dir),
        "-G", _CMAKE_GENERATOR,
        f"-DCMAKE_BUILD_TYPE={config}",
        f"-DBOARD_NAME={args.board}",
        f"-DDRIVER_PROFILE={args.drivers}",
        f"-DHIL_TEST={build_tests}"
    ]
    print(f"Configuring …  ({' '.join(cmake_args)})")
    try:
        subprocess.run(cmake_args, stdout=cmake_cfg_log.open("w"),
                       stderr=subprocess.STDOUT, check=True, env=env)
    except subprocess.CalledProcessError:
        print(f"ERROR: CMake configure failed — see {cmake_cfg_log}")
        sys.exit(1)

    print("Building …")
    try:
        subprocess.run(["cmake", "--build", str(output_dir), "-j"],
                       stdout=cmake_build_log.open("w"),
                       stderr=subprocess.STDOUT, check=True, env=env)
    except subprocess.CalledProcessError:
        print(f"ERROR: CMake build failed — see {cmake_build_log}")
        sys.exit(1)

    print(f"Build complete.  Artifacts: {output_dir}")



# ══════════════════════════════════════════════════════════════════════════════
#  flash
# ══════════════════════════════════════════════════════════════════════════════

def _flash(board: str, config: str, run_tests: bool, openocd: str) -> None:
    m7     = BUILD_ROOT / board / config / "cm7.elf"
    m4     = BUILD_ROOT / board / config / "cm4.elf"
    target = _TARGET_DUAL
    cmds   = [
        "init",
        "reset init",
        "reset halt",
        f"flash write_image erase {m7}",
        f"flash write_image erase {m4}",
        "reset run",
        "exit",
    ]

    cmd = ["sudo", openocd, "-f", _IFACE_CFG, "-f", target]
    if PLATFORM == "win32":
        cmd = cmd[1:]  # Remove "sudo" on Windows
    for c in cmds:
        cmd += ["-c", c]

    label = "hw-test firmware" if run_tests else f"{config} firmware"
    print(f"Flashing {label} to {board} …")
    result = subprocess.run(cmd)
    if result.returncode != 0:
        print(f"ERROR: openocd exited with code {result.returncode}")
        sys.exit(result.returncode)
    print("Flash complete.\n")


def _run_tests(port: str, baud: int, timeout: float) -> None:
    try:
        import serial
        import serial.tools.list_ports
    except ImportError:
        print("ERROR: pyserial not installed.  Run: pip install pyserial")
        sys.exit(2)

    print(f"Opening {port} at {baud} baud …")
    # print([(port.device, port.description) for port in serial.tools.list_ports.comports()])
    try:
        ser = serial.Serial(port, baud, timeout=20)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(2)

    # Unity output format:
    #   file.c:42:test_name:PASS
    #   file.c:42:test_name:FAIL:Expected X Was Y
    #   -----------------------
    #   N Tests N Failures N Ignored
    #   OK  (or FAIL)
    _result_re  = re.compile(r'^.+:\d+:.+:(PASS|FAIL.*)$')
    _summary_re = re.compile(r'^(\d+) Tests (\d+) Failures \d+ Ignored$')

    failures = 0
    total = 0
    found_summary = False
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        try:
            line = raw.decode("ascii", errors="replace").strip()
        except Exception:
            continue
        if not line:
            continue

        print(line)

        if _result_re.match(line):
            total += 1
            if ":FAIL" in line:
                failures += 1
        elif m := _summary_re.match(line):
            total    = int(m.group(1))
            failures = int(m.group(2))
            found_summary = True
            break

    ser.close()

    if not found_summary:
        print(f"\nERROR: Timed out after {timeout}s waiting for Unity summary line.")
        sys.exit(2)

    passed = total - failures
    print(f"\n{'='*40}\nResult: {passed}/{total} tests passed\n{'='*40}")
    sys.exit(0 if failures == 0 else 1)


def cmd_flash(args: argparse.Namespace) -> None:
    config     = 'Debug'
    run_tests  = False

    for ch in args.flags.lower():
        if   ch == 'd': config    = 'Debug'
        elif ch == 'r': config    = 'Release'
        elif ch == 't': run_tests = True
        else:
            print(f"ERROR: Unknown flag '{ch}'. Valid: d=Debug, r=Release, t=run tests")
            sys.exit(1)

    _flash(args.board, config, run_tests, _OPENOCD_PATH)
    if run_tests:
        _run_tests(args.port, args.baud, args.timeout)


# ══════════════════════════════════════════════════════════════════════════════
#  CLI wiring
# ══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    root = argparse.ArgumentParser(
        prog="board.py",
        description="Flapjack board management (install / build / flash)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = root.add_subparsers(dest="command", metavar="<command>")
    sub.required = True

    # ── install ────────────────────────────────────────────────────────────────
    sub.add_parser("install", help="Install SDK tools (ARM GCC, LLVM, OpenOCD)")

    # ── build ──────────────────────────────────────────────────────────────────
    p_build = sub.add_parser("build", help="Configure and build firmware",
                              formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p_build.add_argument("--board", "-b", required=True, choices=BOARDS,
                         help="Target board")
    p_build.add_argument("--flags", "-f", default="d", metavar="FLAGS",
                         help="Build flags (combine freely): d=Debug, r=Release, "
                              "c=clean, t=HIL tests, g=regen proto+umsg")
    p_build.add_argument("--drivers", "-D", default="default", metavar="PROFILE",
                         help="Driver profile in Firmware/target/drivers/ "
                              "(e.g. default=real hardware, sim=no hardware)")

    # ── flash ──────────────────────────────────────────────────────────────────
    p_flash = sub.add_parser("flash", help="Flash firmware and optionally run HIL tests",
                              formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p_flash.add_argument("--board", "-b", required=True, choices=BOARDS,
                         help="Target board")
    p_flash.add_argument("--flags", "-f", default="d", metavar="FLAGS",
                         help="Flash flags (combine freely): d=Debug, r=Release, t=run tests")
    p_flash.add_argument("--port", default="/dev/ttyACM0",
                         help="Serial port for test output")
    p_flash.add_argument("--baud", type=int, default=230400,
                         help="Baud rate")
    p_flash.add_argument("--timeout", type=float, default=30,
                         help="Seconds to wait for RESULTS line")
    p_flash.add_argument("--openocd", default=None,
                         help="Path to openocd binary (auto-detected if omitted)")

    args = root.parse_args()
    {"install": cmd_install, "build": cmd_build, "flash": cmd_flash}[args.command](args)


if __name__ == "__main__":
    main()
