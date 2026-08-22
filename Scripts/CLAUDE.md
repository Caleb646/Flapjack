# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Common Commands

All commands run from the project root.

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

# Install the Python tooling deps (base + optional extras)
pip install -e ".[sim,gui,gen]"

# Regenerate protobuf + umsg sources (no toolchain/build)
python3 Scripts/board.py gen

# Host side of the sim link — JSBSim HIL bridge. Naming a real device turns the
# SIL defaults off, so a board run wires only what you ask for.
python3 Scripts/board.py sim --port /dev/ttyUSB0 --dry-run

# SIL: emulate the CM7 in Renode instead of using a board (two terminals).
# `renode` takes the place of `flash`. USART1 -> port 4000 (sim link: actuators,
# telemetry, logs, shell), USART3 -> port 4001 (CRSF RC; without it the FC never
# arms), USART2 -> port 4002 (NMEA GPS; optional - without it the FC just never
# sees a fix), and 4010/4011/4012 carry samples into the emulated IMU, mag and
# baro, which the FC reads through its real drivers over emulated SPI.
# All six wires default to the SIL sockets, so the bridge needs no flags at all.
python3 Scripts/board.py build -b flapjack-v1 -D sim --single-core
python3 Scripts/board.py renode -b flapjack-v1
python3 Scripts/board.py sim

# Fly a scripted flight plan (exits 0 on pass, 1 on failed checks)
python3 Scripts/board.py sim --plan Scripts/sim/plans/hover.yaml

# Host unit tests
cmake -S Tests -B Build/UnitTest -G "MinGW Makefiles" && ctest --test-dir Build/UnitTest

# Flight GUI (telemetry + command console)
python3 Scripts/board.py gui

# Watch a SIL run in the GUI. The bridge owns the FC's wire, so the GUI cannot
# read the board during a sim; it publishes FDM truth, the FC's estimate and the
# FC's servo/motor commands as JSON over UDP instead. On by default (--gui-port,
# 5005; 0 disables), so the sim command above needs no extra flag - just hit
# Listen in the GUI's "Sim Graphs" tab. The 3D viewer follows the same feed,
# flying on the FC's own estimated attitude and altitude - flip its Source to
# Truth to watch JSBSim's instead, or Pop Out for a window of its own. The same
# link runs the other way: tick "Send Sticks"
# in that tab's RC Sticks panel to fly throttle/roll/pitch/yaw by hand. There is
# no mode switch - the FC has one flight mode, always active, holding altitude on
# the throttle stick and attitude on roll/pitch. The bridge overrides whatever it
# would otherwise send with those sticks, and takes its own back 0.5 s after the
# GUI goes quiet.
python3 Scripts/board.py gui
```

Supported boards: `nucleo-h747zi`, `flapjack-v1`. Build configs: `Debug`, `Release`.

Build artifacts land in `Build/<board>/<config>/` as `cm7.elf` and `cm4.elf`. CMake logs go to `cmake_configure.log` and `cmake_build.log` in that directory.

## Architecture

### Script roles

`Scripts/` is the home for all of the project's Python tooling. `board.py` is the single CLI entry point (`python Scripts/board.py <cmd>`); it runs with `Scripts/` on `sys.path`, so the subpackages import as top-level (`proto`, `link`, `sim`, `gui`). Dependencies are pinned in the repo-root `pyproject.toml` (base + `[sim]`/`[gui]`/`[gen]` extras).

- **`board.py`** — single CLI entry point. Subcommands: `install`, `build`, `flash` (firmware); `gen` (regenerate proto + umsg, no toolchain/build); `renode` (boot the firmware under the Renode CM7 emulator); `sim` (JSBSim HIL bridge — remaining args are forwarded to it); `gui` (PyQt flight GUI). `sim`/`gui` imports are lazy so `build`/`flash` never require their heavy deps. `build` calls `install` automatically if the toolchain is missing.
- **`renode/`** — Renode platform overlay (`flapjack_h7_cm7.repl`), machine script (`flapjack_sil.resc`) and the emulated sensor parts (`BMI323.cs`, `MMC5983.cs`, `BMP390.cs`, sharing `SamplePushListener.cs`) for the single-core CM7 SIL. Driven by `board.py renode`, which exposes USART1 as the sim link on port 4000, USART3 as the CRSF RC link on 4001, and USART2 as the NMEA GPS link on 4002. The three sensor models open their own sockets (4010/4011/4012) from their `port:` properties in the `.repl`; mag and baro share SPI5 behind an `SPIMultiplexer`. See `SilResearch.md` §2 for why each overlay entry exists.
- **`proto/`** — the single home for generated protobuf Python stubs (`flapjack_pb2.py`, `sim_pb2.py`), emitted here by `build -f g`; all tools import `from proto import …`.
- **`link/`** — shared host-side link plumbing: `framing.py` (crc8/frame/deframe, mirrors `sim_link.c`) and `serial_io.py` (pyserial helpers).
- **`sim/`** — the JSBSim HIL bridge (`bridge.py`), the CRSF encoder (`crsf.py`), the NMEA encoder
  (`nmea.py`), the flight-plan loader (`plan.py`) with its plans in `sim/plans/*.yaml`, plus the
  `jsbsim/` models and the sensor systems vendored into `jsbsim/systems/`. See `sim/README.md`.
  **Nothing reaches the FC pre-decoded any more**, so the SIL exercises the real parse and
  driver paths instead of injecting values. RC arrives as CRSF on the RX UART (`--rc-port`, the
  only RC path — without it the FC never arms) and GPS as NMEA on the GPS UART (`--gps-port`,
  optional). IMU, mag and baro are pushed into emulated parts (`--imu-port`/`--mag-port`/
  `--baro-port`) and read back through the real `bmi323`, `mmc5983` and `bmp390` drivers over
  emulated SPI. The sim link itself is now FC→PC only, apart from the shell.
  **All six wires default to their SIL sockets** (4000/4001/4002 and 4010/4011/4012, see
  `resolve_ports()`), so a SIL run is a bare `board.py sim`. Naming a real device for `--port`
  turns the defaults off, because the companion wires on a board are physical ports nobody can
  guess. Pass `none` to any one of them — `--gps-port none` — to leave that wire off, which is
  how the "no fix" and "no receiver" paths stay testable.
- **`gui/`** — the PyQt flight GUI (`app.py`, `conf.py`, `data/`). Two independent
  input interfaces: the serial link to a real FC (`<json>` telemetry over
  `QSerialPort`), and the sim link (`SimLink`, a UDP feed from the bridge's
  `--gui-port`) which supplies the "Sim Graphs" tab and the 3D viewer during a
  SIL run. They are never live at once - the bridge holds the FC's wire. The
  3D viewer picks between the feed's two poses, defaulting to the FC's estimate,
  and honours `nav_valid` one axis at a time: an axis the FC does not yet vouch
  for holds its last good value instead of being drawn as zero. It also pops out
  into a window of its own, which is why `main()` sets `AA_ShareOpenGLContexts` -
  reparenting a `QOpenGLWidget` recreates its context, and unshared VBOs die with
  the old one. The sim link is bidirectional: the GUI answers the feed with
  stick positions, which the bridge folds into the CRSF it is already sending. Arming is not
  among them - that stays with the bridge's own gesture.
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
