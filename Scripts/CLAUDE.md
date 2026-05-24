# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Common Commands

All commands run from the project root (`/home/caleb/projects/Flapjack`).

```bash
# Install toolchain (one-time; downloads ARM GCC, LLVM, OpenOCD into Tools/)
python3 Scripts/board.py install

# Build firmware (Debug is default)
python3 Scripts/board.py build --board nucleo-h747zi
python3 Scripts/board.py build --board nucleo-h747zi --config Release

# Flash to hardware via ST-Link
python3 Scripts/board.py flash --board nucleo-h747zi

# Build and run HIL tests
python3 Scripts/board.py build --board nucleo-h747zi --build-tests
python3 Scripts/board.py flash --board nucleo-h747zi --run-tests --port /dev/ttyACM0 --baud 230400
```

Supported boards: `nucleo-h747zi`, `flapjack-v1`. Build configs: `Debug`, `Release`.

Build artifacts land in `Build/<board>/<config>/` as `cm7.elf` and `cm4.elf`. CMake logs go to `cmake_configure.log` and `cmake_build.log` in that directory.

## Architecture

### Script roles

- **`board.py`** — single CLI entry point for all build/flash/test operations. The three subcommands (`install`, `build`, `flash`) are independent; `build` calls `install` automatically if the toolchain is missing.
- **`utils.py`** — two helpers used by other scripts: `is_installed(binary)` (shutil.which) and `run_command(cmd, ...)` (subprocess, exits on failure).
- **`stm32h747_dual_core.cfg`** — OpenOCD config for the dual-core STM32H747. Used directly by `flash`. Enables `DUAL_BANK` and `DUAL_CORE`, resets via `connect_assert_srst`, flashes both CM7 and CM4 banks.

### HIL test flow

HIL tests live in `Tests/HIL/`. The firmware is compiled with `HIL_TEST=True` (CMake flag set by `--build-tests`). On boot, the board runs Unity test cases and streams results over UART in the format:

```
file.c:42:test_name:PASS
file.c:43:test_name:FAIL:Expected X Was Y
N Tests M Failures 0 Ignored
```

`flash --run-tests` opens the serial port, streams output to stdout, and exits 0 (all pass) or 1 (failures). Requires `pip install pyserial`.

### Toolchain layout (after install)

```
Tools/
  arm-none-eabi-gcc/   # xPack ARM GNU GCC
  llvm/                # LLVM/Clang
  openocd/             # xPack OpenOCD (bin/openocd)
```

`flash` auto-detects OpenOCD at `Tools/openocd/bin/openocd`, falling back to PATH.
