import os
import sys
import subprocess
import urllib.request
import tarfile
from pathlib import Path

PLATFORM = sys.platform

FJ_PROJECT_ROOT = str(Path(__file__).parent.parent)
FJ_TOOLS_ROOT_DIR_PATH = os.path.join(FJ_PROJECT_ROOT, "Tools")

# ── Windows / MSYS2 ────────────────────────────────────────────────────────────
MSYS_DIR = os.path.join(FJ_TOOLS_ROOT_DIR_PATH, "msys")
MSYS_EXE_FNAME = "msys2-x86_64-20250830.exe"
MSYS_EXE_URL = f"https://repo.msys2.org/distrib/x86_64/{MSYS_EXE_FNAME}"
MINGW64_GNU_BIN_PATH = os.path.join(MSYS_DIR, "mingw64", "bin")

# ── Linux ──────────────────────────────────────────────────────────────────────
# ARM GNU Toolchain — xPack distribution (GitHub releases, no browser redirect)
ARM_GNU_VERSION = "15.2.1-1.1"
ARM_GNU_TARBALL = f"xpack-arm-none-eabi-gcc-{ARM_GNU_VERSION}-linux-x64.tar.gz"
ARM_GNU_URL = (
    f"https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download"
    f"/v{ARM_GNU_VERSION}/{ARM_GNU_TARBALL}"
)
LINUX_ARM_GNU_DIR = os.path.join(FJ_TOOLS_ROOT_DIR_PATH, "arm-gnu")
LINUX_ARM_GNU_BIN = os.path.join(LINUX_ARM_GNU_DIR, "bin")

# LLVM/Clang — pre-built binaries for x86_64 Linux
LLVM_VERSION = "22.1.5"
LLVM_DIRNAME = f"LLVM-{LLVM_VERSION}-Linux-X64"
LLVM_TARBALL = f"{LLVM_DIRNAME}.tar.xz"
LLVM_URL = (
    f"https://github.com/llvm/llvm-project/releases/download"
    f"/llvmorg-{LLVM_VERSION}/{LLVM_TARBALL}"
)
LLVM_DIR = os.path.join(FJ_TOOLS_ROOT_DIR_PATH, "llvm")
LLVM_BIN = os.path.join(LLVM_DIR, "bin")

# OpenOCD — xPack distribution (GitHub releases)
OPENOCD_VERSION = "0.12.0-7"
OPENOCD_DIRNAME = f"xpack-openocd-{OPENOCD_VERSION}"
OPENOCD_TARBALL = f"{OPENOCD_DIRNAME}-linux-x64.tar.gz"
OPENOCD_URL = (
    f"https://github.com/xpack-dev-tools/openocd-xpack/releases/download"
    f"/v{OPENOCD_VERSION}/{OPENOCD_TARBALL}"
)
OPENOCD_DIR = os.path.join(FJ_TOOLS_ROOT_DIR_PATH, "openocd")
OPENOCD_BIN = os.path.join(OPENOCD_DIR, "bin")

# ── Shared ─────────────────────────────────────────────────────────────────────
ARM_GNU_PREFIX = "arm-none-eabi"
ARM_GNU_BIN_PATH = MINGW64_GNU_BIN_PATH if PLATFORM == "win32" else LINUX_ARM_GNU_BIN


# ── Windows helpers ────────────────────────────────────────────────────────────

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

    print("Installing OpenOCD...")
    run_msys_command(
        'pacman -S --needed --noconfirm mingw-w64-x86_64-openocd',
        "Failed to install OpenOCD"
    )


# ── Linux helpers ──────────────────────────────────────────────────────────────

def _download_progress(block_num, block_size, total_size):
    if total_size > 0:
        pct = min(100, block_num * block_size * 100 // total_size)
        print(f"\r  {pct}%", end="", flush=True)


def download_and_extract(url, tarball_name, dest_dir, label):
    if os.path.isdir(dest_dir):
        print(f"{label} already installed at: {dest_dir}")
        print("Skipping download and installation.")
        return

    os.makedirs(FJ_TOOLS_ROOT_DIR_PATH, exist_ok=True)
    tarball_path = os.path.join(FJ_TOOLS_ROOT_DIR_PATH, tarball_name)

    print(f"Downloading {label}...")
    print(f"  URL: {url}")
    try:
        urllib.request.urlretrieve(url, tarball_path, reporthook=_download_progress)
        print()
    except Exception as e:
        print(f"\nERROR: Failed to download {label}: {e}")
        sys.exit(1)

    print(f"Extracting {label}...")
    try:
        with tarfile.open(tarball_path) as tf:
            top_level = tf.getnames()[0].split('/')[0]
            tf.extractall(FJ_TOOLS_ROOT_DIR_PATH)
    except Exception as e:
        print(f"ERROR: Failed to extract {label}: {e}")
        sys.exit(1)

    extracted_path = os.path.join(FJ_TOOLS_ROOT_DIR_PATH, top_level)
    if extracted_path != dest_dir:
        os.rename(extracted_path, dest_dir)

    os.remove(tarball_path)
    print(f"Done: {label} installed at {dest_dir}")


def install_linux_tools():
    download_and_extract(ARM_GNU_URL, ARM_GNU_TARBALL, LINUX_ARM_GNU_DIR, "ARM GNU Toolchain")
    download_and_extract(LLVM_URL, LLVM_TARBALL, LLVM_DIR, "LLVM/Clang")
    download_and_extract(OPENOCD_URL, OPENOCD_TARBALL, OPENOCD_DIR, "OpenOCD")


# ── Verification ───────────────────────────────────────────────────────────────

def _check_paths(required_tools):
    all_found = True
    for name, path in required_tools.items():
        if os.path.exists(path):
            print(f"  OK:      {name}")
        else:
            print(f"  MISSING: {name} ({path})")
            all_found = False
    return all_found


def verify_tools_windows():
    print("Verifying installed tools...")
    return _check_paths({
        "MSYS2 directory":   MSYS_DIR,
        "MinGW64 bin":       MINGW64_GNU_BIN_PATH,
        "mingw32-make":      os.path.join(MINGW64_GNU_BIN_PATH, "mingw32-make.exe"),
        "ARM GCC":           os.path.join(MINGW64_GNU_BIN_PATH, f"{ARM_GNU_PREFIX}-gcc.exe"),
        "ARM G++":           os.path.join(MINGW64_GNU_BIN_PATH, f"{ARM_GNU_PREFIX}-g++.exe"),
        "ARM objcopy":       os.path.join(MINGW64_GNU_BIN_PATH, f"{ARM_GNU_PREFIX}-objcopy.exe"),
        "ARM size":          os.path.join(MINGW64_GNU_BIN_PATH, f"{ARM_GNU_PREFIX}-size.exe"),
    })


def verify_tools_linux():
    print("Verifying installed tools...")
    return _check_paths({
        "ARM GNU toolchain":     LINUX_ARM_GNU_DIR,
        "arm-none-eabi-gcc":     os.path.join(LINUX_ARM_GNU_BIN, f"{ARM_GNU_PREFIX}-gcc"),
        "arm-none-eabi-g++":     os.path.join(LINUX_ARM_GNU_BIN, f"{ARM_GNU_PREFIX}-g++"),
        "arm-none-eabi-gdb":     os.path.join(LINUX_ARM_GNU_BIN, f"{ARM_GNU_PREFIX}-gdb"),
        "arm-none-eabi-objcopy": os.path.join(LINUX_ARM_GNU_BIN, f"{ARM_GNU_PREFIX}-objcopy"),
        "arm-none-eabi-size":    os.path.join(LINUX_ARM_GNU_BIN, f"{ARM_GNU_PREFIX}-size"),
        "LLVM/Clang":            LLVM_DIR,
        "clang":                 os.path.join(LLVM_BIN, "clang"),
        "clang++":               os.path.join(LLVM_BIN, "clang++"),
        "clang-tidy":            os.path.join(LLVM_BIN, "clang-tidy"),
        "openocd":               os.path.join(OPENOCD_BIN, "openocd"),
    })


def verify_tools():
    if PLATFORM == "win32":
        return verify_tools_windows()
    return verify_tools_linux()


# ── Entry point ────────────────────────────────────────────────────────────────

def main():

    print("FlapJack SDK Installation")
    print(f"Project Root: {FJ_PROJECT_ROOT}")
    print(f"Platform:     {PLATFORM}")

    if PLATFORM == "win32":
        install_msys()
    elif PLATFORM.startswith("linux"):
        install_linux_tools()
    else:
        print(f"ERROR: Unsupported platform '{PLATFORM}'. Supported: win32, linux.")
        sys.exit(1)

    if verify_tools():
        print("\nSDK installation completed successfully!")
    else:
        print("\nSDK installation completed with errors — some required tools are missing.")
        sys.exit(1)


if __name__ == "__main__":
    main()
