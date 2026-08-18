#!/usr/bin/env python3
"""
Flapjack tooling CLI (single entry point for the project's Python tools).

Commands
--------
  install           Install SDK tools (ARM GCC, LLVM, OpenOCD)
  build             Configure and build firmware
  flash             Flash firmware; optionally run HIL tests
  gen               Regenerate protobuf + umsg sources (no toolchain/build)
  renode            Boot the firmware under the Renode CM7 emulator (SIL)
  sim               Run the JSBSim HIL bridge (host side of the sim link)
  gui               Launch the flight GUI (telemetry + command console)

Examples
--------
  python3 Scripts/board.py install
  python3 Scripts/board.py build --board nucleo-h747zi
  python3 Scripts/board.py build --board nucleo-h747zi -f r
  python3 Scripts/board.py build --board nucleo-h747zi -f dc
  python3 Scripts/board.py build --board nucleo-h747zi -f rc
  python3 Scripts/board.py build --board nucleo-h747zi -f gcr
  python3 Scripts/board.py build --board flapjack-v1 -D sim
  python3 Scripts/board.py flash --board nucleo-h747zi
  python3 Scripts/board.py flash --board nucleo-h747zi -f r
  python3 Scripts/board.py flash --board nucleo-h747zi -f t
  python3 Scripts/board.py flash --board nucleo-h747zi -f dt --port /dev/ttyUSB0 --baud 115200
  python3 Scripts/board.py gen
  python3 Scripts/board.py sim --port COM7 --dry-run
  python3 Scripts/board.py gui

  # SIL: emulate the CM7 instead of using a board. Two terminals -
  # `renode` replaces `flash`, and `sim` is unchanged apart from --port.
  python3 Scripts/board.py build -b flapjack-v1 -D sim --single-core
  python3 Scripts/board.py renode -b flapjack-v1
  python3 Scripts/board.py sim --port socket://localhost:4000 --rc-port socket://localhost:4001 --rate 400

  # USART1 -> 4000 is the sim link; USART3 -> 4001 carries real CRSF to the RX
  # UART and is the FC's only RC input - without --rc-port it never arms.
  # Fly a scripted plan (exit 0 = checks passed, 1 = failed):
  python3 Scripts/board.py sim --port socket://localhost:4000 --rc-port socket://localhost:4001       --plan Scripts/sim/plans/hover.yaml
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
SCRIPTS_ROOT  = Path(__file__).parent
BUILD_ROOT    = PROJECT_ROOT / "Build"


def output_dir(board: str, config: str, single_core: bool) -> Path:
    """Artifact directory for one (board, config, core-count) combination.

    Single- and dual-core images used to share Build/<board>/<config>/, so a
    stale cm4.elf could sit beside a freshly built single-core cm7.elf and
    `flash` would happily write both. Keeping them apart makes the tree say
    which kind of image it holds.
    """
    return BUILD_ROOT / board / (f"{config}-single-core" if single_core else config)
TOOLS_ROOT    = PROJECT_ROOT / "Tools"

BOARDS  = ["nucleo-h747zi", "flapjack-v1"]
CONFIGS = ["Debug", "Release"]

# Mirrors CFG_LOG_LEVEL_* in Firmware/target/cfgs/cfg.h.
LOG_LEVELS = {"none": 0, "error": 1, "warn": 2, "info": 3, "debug": 4}

# ── Generation ─────────────────────────────────────────────────────

UMSG_FILES_DIR          = PROJECT_ROOT / "Firmware" / "msgs" / "umsg" / "defs"
UMSG_OUTPUT_DIR         = PROJECT_ROOT / "Firmware" / "msgs" / "umsg"

PROTO_FILES_DIR         = PROJECT_ROOT / "Firmware" / "msgs" / "proto" / "defs"
PROTO_OUTPUT_DIR        = PROJECT_ROOT / "Firmware" / "msgs" / "proto"
PROTO_PY_OUTPUT_DIR     = SCRIPTS_ROOT / "proto"

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
_CMAKE_GENERATOR = "Unix Makefiles"
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
            # Python stubs via the venv's grpc_tools so gencode matches the
            # installed protobuf runtime (avoids the gencode/runtime VersionError).
            subprocess.run(
                [str(VENV_PYTHON), "-m", "grpc_tools.protoc", str(proto_file),
                 f"-I{PROTO_FILES_DIR}", f"--python_out={PROTO_PY_OUTPUT_DIR}"],
                check=True, capture_output=True, text=True, env=os.environ.copy(),
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


def _cached_log_level(out_dir: Path):
    """CFG_LOG_LEVEL the existing build tree was configured with, or None."""
    cache = out_dir / "CMakeCache.txt"
    if not cache.exists():
        return None
    for line in cache.read_text(errors="ignore").splitlines():
        if line.startswith("CFG_LOG_LEVEL:"):
            try:
                return int(line.split("=", 1)[1])
            except (IndexError, ValueError):
                return None
    return None


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

    cores = "single-core (cm7)" if args.single_core else "dual-core"
    print(f"Flapjack Firmware Build  |  board: {args.board}  |  config: {config}  |  drivers: {args.drivers}  |  {cores}\n")
    _verify_toolchain()

    if clean or regen:
        _generate_proto()
        _generate_umsg()

    env = os.environ.copy()
    env["PATH"] = str(_HOST_ARM_BIN) + os.pathsep + env["PATH"]

    out_dir = output_dir(args.board, config, args.single_core)

    log_level = LOG_LEVELS[getattr(args, "log_level", "debug")]

    # A changed -D does not reliably force a recompile here: CMake rewrites
    # flags.make during configure, but MinGW Make compares mtimes at 1-second
    # resolution, so objects written in the same second look up to date and the
    # link silently reuses them. That yields an image built at the PREVIOUS log
    # level. Detect the change against the cache and clean instead.
    if not clean and _cached_log_level(out_dir) not in (None, log_level):
        print("Log level changed - forcing a clean rebuild (logs are gated at compile time)")
        clean = True

    if clean and out_dir.exists():
        print(f"Cleaning {out_dir} …")
        shutil.rmtree(out_dir)

    cmake_cfg_log   = out_dir / "cmake_configure.log"
    cmake_build_log = out_dir / "cmake_build.log"
    os.makedirs(out_dir, exist_ok=True)

    cmake_args = [
        "cmake",
        "-S", str(PROJECT_ROOT),
        "-B", str(out_dir),
        "-G", _CMAKE_GENERATOR,
        f"-DCMAKE_BUILD_TYPE={config}",
        f"-DBOARD_NAME={args.board}",
        f"-DDRIVER_PROFILE={args.drivers}",
        f"-DHIL_TEST={build_tests}",
        f"-DSINGLE_CORE={'ON' if args.single_core else 'OFF'}",
    ]
    cmake_args.append(f"-DCFG_LOG_LEVEL={LOG_LEVELS[getattr(args, 'log_level', 'debug')]}")
    print(f"Configuring …  ({' '.join(cmake_args)})")
    try:
        subprocess.run(cmake_args, stdout=cmake_cfg_log.open("w"),
                       stderr=subprocess.STDOUT, check=True, env=env)
    except subprocess.CalledProcessError:
        print(f"ERROR: CMake configure failed — see {cmake_cfg_log}")
        sys.exit(1)

    print("Building …")
    try:
        subprocess.run(["cmake", "--build", str(out_dir), "-j"],
                       stdout=cmake_build_log.open("w"),
                       stderr=subprocess.STDOUT, check=True, env=env)
    except subprocess.CalledProcessError:
        print(f"ERROR: CMake build failed — see {cmake_build_log}")
        sys.exit(1)

    print(f"Build complete.  Artifacts: {out_dir}")



# ══════════════════════════════════════════════════════════════════════════════
#  flash
# ══════════════════════════════════════════════════════════════════════════════

def _flash(board: str, config: str, run_tests: bool, openocd: str) -> None:
    # Flashing writes both cores, so it wants the dual-core tree.
    out    = output_dir(board, config, single_core=False)
    m7     = out / "cm7.elf"
    m4     = out / "cm4.elf"
    for elf in (m7, m4):
        if not elf.exists():
            print(f"ERROR: {elf} not found.")
            print(f"       Build it first: python Scripts/board.py build -b {board}")
            sys.exit(1)
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
#  gen  (regenerate protobuf + umsg sources, no build)
# ══════════════════════════════════════════════════════════════════════════════

def cmd_gen(_: argparse.Namespace) -> None:
    # Same generation `build -f g` does, but without the ARM toolchain or a build.
    # Needs the venv + [gen] extra (grpc_tools, nanopb, umsg-gen).
    _generate_proto()
    _generate_umsg()
    print("Generation complete.")


# ══════════════════════════════════════════════════════════════════════════════
#  renode
# ══════════════════════════════════════════════════════════════════════════════

RENODE_DIR    = TOOLS_ROOT / "renode"
RENODE_SCRIPT = SCRIPTS_ROOT / "renode" / "flapjack_sil.resc"


def _renode_binary() -> str:
    exe = RENODE_DIR / ("renode.exe" if PLATFORM == "win32" else "renode")
    if exe.exists():
        return str(exe)
    found = shutil.which("renode")
    if found:
        return found
    print(f"ERROR: Renode not found at {exe}, and 'renode' is not on PATH.")
    print("       Get the portable package from https://builds.renode.io and unpack it to Tools/renode/")
    sys.exit(1)


def cmd_renode(args: argparse.Namespace) -> None:
    config = "Release" if "r" in args.flags else "Debug"
    # Renode models the CM7 only, so it always wants the single-core tree; a
    # dual-core image spins forever in main() waiting on s_IsCM4Ready.
    out_dir = output_dir(args.board, config, single_core=True)
    elf = Path(args.elf) if args.elf else out_dir / "cm7.elf"

    if not elf.exists():
        print(f"ERROR: {elf} not found.")
        print(f"       Build it first: python Scripts/board.py build -b {args.board} -D sim --single-core")
        sys.exit(1)

    # Belt and braces: the directory says single-core, so confirm the cache agrees.
    cache = out_dir / "CMakeCache.txt"
    if args.elf is None and cache.exists() and "SINGLE_CORE:BOOL=ON" not in cache.read_text():
        print(f"ERROR: {elf} was built dual-core (SINGLE_CORE=OFF).")
        print("       Renode emulates the CM7 only; a dual-core image hangs in main() waiting for CM4.")
        print(f"       Rebuild: python Scripts/board.py build -b {args.board} -D sim --single-core")
        sys.exit(1)

    # $bin is declared with '?=' in the .resc, so setting it first wins.
    # usart1 carries the sim link (sensors/actuators/telemetry/logs); usart3 is
    # the RX UART, so the bridge can feed real CRSF frames to Rx_Task there; and
    # usart2 is the GPS UART, carrying real NMEA sentences to Gps_Task.
    monitor_cmds = "; ".join([
        f"$bin=@{elf.resolve().as_posix()}",
        f"i @{RENODE_SCRIPT.resolve().as_posix()}",
        f'emulation CreateServerSocketTerminal {args.port} "simlink" false',
        "connector Connect sysbus.usart1 simlink",
        f'emulation CreateServerSocketTerminal {args.rc_port} "rclink" false',
        "connector Connect sysbus.usart3 rclink",
        f'emulation CreateServerSocketTerminal {args.gps_port} "gpslink" false',
        "connector Connect sysbus.usart2 gpslink",
        "start",
    ])

    # -P (monitor on a socket) rather than --console: with stdin not a TTY the
    # console monitor reads EOF and Renode quits immediately.
    cmd = [_renode_binary(), "-P", str(args.monitor_port), "--plain"]
    if not args.gui:
        cmd.append("--disable-xwt")
    cmd += ["-e", monitor_cmds]

    print(f"Renode  |  board: {args.board}  |  config: {config}  |  {elf.name}")
    print(f"  sim link : socket://localhost:{args.port}")
    print(f"  rc link  : socket://localhost:{args.rc_port}")
    print(f"  gps link : socket://localhost:{args.gps_port}")
    print(f"  monitor  : localhost:{args.monitor_port}")
    print(f"  bridge   : python Scripts/board.py sim --port socket://localhost:{args.port} "
          f"--rc-port socket://localhost:{args.rc_port} "
          f"--gps-port socket://localhost:{args.gps_port} --rate 400")
    print("  Ctrl-C to stop.\n")
    try:
        subprocess.run(cmd, cwd=PROJECT_ROOT, check=False)
    except KeyboardInterrupt:
        pass


# ══════════════════════════════════════════════════════════════════════════════
#  CLI wiring
# ══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    # `sim` and `gui` forward their remaining args verbatim to those tools (lazy
    # import keeps their heavy deps off the build/flash path). argparse REMAINDER
    # mishandles a leading option like `sim --port`, so intercept them here before
    # full parsing; they are still registered below so `--help` lists them.
    if len(sys.argv) >= 2 and sys.argv[1] == "sim":
        from sim import bridge
        return bridge.main(sys.argv[2:])
    if len(sys.argv) >= 2 and sys.argv[1] == "gui":
        from gui import app
        return app.main()

    root = argparse.ArgumentParser(
        prog="board.py",
        description="Flapjack tooling (install / build / flash / sim / gui)",
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
                         help="Driver profile in Firmware/drivers/profiles/ "
                              "(e.g. default=real hardware, sim=no hardware)")
    p_build.add_argument("--single-core", action="store_true",
                         help="Build a single-core CM7 image (no cm4.elf); for Renode SIL")
    # Always passed to CMake, never left unset: -D values are cached, so an
    # omitted flag would silently inherit the previous build's level.
    p_build.add_argument("--log-level", choices=list(LOG_LEVELS), default="debug",
                         help="Compile out logs above this level. Anything above it is removed "
                              "by the preprocessor, not suppressed at runtime. Note 'debug' "
                              "also gates LOG_DATA, which is what the GUI plots.")

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

    # ── gen ──────────────────────────────────────────────────────────────────────
    sub.add_parser("gen", help="Regenerate protobuf + umsg sources (no toolchain/build)")

    # ── renode ─────────────────────────────────────────────────────────────────
    p_renode = sub.add_parser("renode", help="Boot the firmware under the Renode CM7 emulator",
                              formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p_renode.add_argument("--board", "-b", default="flapjack-v1", choices=BOARDS,
                          help="Target board")
    p_renode.add_argument("--flags", "-f", default="d", metavar="FLAGS",
                          help="d=Debug, r=Release (selects the Build/ subdirectory)")
    p_renode.add_argument("--port", type=int, default=4000,
                          help="TCP port exposing USART1 as the sim link")
    p_renode.add_argument("--rc-port", type=int, default=4001,
                          help="TCP port exposing USART3 as the CRSF RC link")
    p_renode.add_argument("--gps-port", type=int, default=4002,
                          help="TCP port exposing USART2 as the NMEA GPS link")
    p_renode.add_argument("--monitor-port", type=int, default=3456,
                          help="TCP port for the Renode monitor")
    p_renode.add_argument("--elf", default=None,
                          help="Override the ELF path (skips the SINGLE_CORE check)")
    p_renode.add_argument("--gui", action="store_true",
                          help="Show the Renode window instead of running headless")

    # ── sim / gui ────────────────────────────────────────────────────────────────
    # Registered for `--help` listing only; both are intercepted at the top of
    # main() so their args pass straight through to the bridge / GUI.
    sub.add_parser("sim", help="Run the JSBSim HIL bridge (args forwarded to it)",
                   add_help=False)
    sub.add_parser("gui", help="Launch the flight GUI (telemetry + command console)")

    args = root.parse_args()
    {"install": cmd_install, "build": cmd_build, "flash": cmd_flash,
     "gen": cmd_gen, "renode": cmd_renode}[args.command](args)


if __name__ == "__main__":
    sys.exit(main())
