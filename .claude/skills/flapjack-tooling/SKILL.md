---
name: flapjack-tooling
description: How and when to build, flash, and test the Flapjack STM32H745 firmware, run the JSBSim hardware-in-the-loop (HIL) sim bridge, launch the flight GUI, and regenerate protobuf/umsg code — all through the single `Scripts/board.py` CLI. Use for any task that compiles/flashes this firmware, touches the sim link (SIM_HIL / bridge.py), the PyQt GUI, or (re)generates message stubs.
---

# Flapjack tooling (`Scripts/board.py`)

`Scripts/board.py` is the **single entry point** for every Python tool in this repo.
Always run it from the **repo root**:

```
python Scripts/board.py <command> [options]
```

Subcommands: `install`, `build`, `flash`, `gen`, `renode`, `sim`, `gui`. `Scripts/` is on `sys.path`,
so its subpackages import as top-level (`proto`, `link`, `sim`, `gui`) — **do not run
the tool modules directly** (e.g. `python Scripts/sim/bridge.py`); go through `board.py`.

## One-time setup

- **Firmware toolchain** (ARM GCC / LLVM / OpenOCD → `Tools/`): `python Scripts/board.py install`
- **Python deps** (pinned in the repo-root `pyproject.toml`): `pip install -e ".[sim,gui,gen]"`.
  Install only the extras a task needs — `sim` = jsbsim, `gui` = PyQt5 stack, `gen` = proto/umsg generators.

## Commands

| Task | Command |
|---|---|
| Build firmware (Debug, real hardware) | `python Scripts/board.py build -b flapjack-v1` |
| Build HIL / sim firmware | `python Scripts/board.py build -b flapjack-v1 -D sim` |
| Release, clean, + regenerate code | `python Scripts/board.py build -b flapjack-v1 -f rcg` |
| Regenerate proto + umsg only (no build) | `python Scripts/board.py gen` |
| Flash to the board (ST-Link) | `python Scripts/board.py flash -b flapjack-v1` |
| Build then run HIL unit tests | `build -b flapjack-v1 -f t` then `flash -b flapjack-v1 -f t --port <PORT> --baud 230400` |
| Run the host unit tests | `cmake -S Tests -B Build/UnitTest -G "MinGW Makefiles"` then `cmake --build Build/UnitTest && ctest --test-dir Build/UnitTest` |
| Bring up the sim link (no FDM) | `python Scripts/board.py sim --port <PORT> --dry-run` |
| Run the full JSBSim sim | `python Scripts/board.py sim --port <PORT>` |
| Launch the flight GUI | `python Scripts/board.py gui` |
| Build for the Renode SIL | `python Scripts/board.py build -b flapjack-v1 -D sim --single-core` |
| Build with logs compiled out | `python Scripts/board.py build -b flapjack-v1 --log-level warn` |
| Boot the firmware in Renode | `python Scripts/board.py renode -b flapjack-v1` |
| Drive the SIL from the bridge | `python Scripts/board.py sim --port socket://localhost:4000 --rc-port socket://localhost:4001 --imu-port socket://localhost:4010 --rate 400` |
| Fly a scripted flight plan | `... sim --port socket://localhost:4000 --rc-port socket://localhost:4001 --plan Scripts/sim/plans/hover.yaml` |

Boards: `flapjack-v1`, `nucleo-h747zi`.

### SIL (software-in-the-loop) via Renode

`renode` stands in for `flash`: it boots `cm7.elf` on an emulated Cortex-M7 and exposes **two**
UARTs as TCP servers, so `sim` connects to sockets instead of COM ports:

* **USART1 -> port 4000** — the sim link: baro, actuators, telemetry, logs and the shell.
* **TCP 4010** — the emulated BMI323. `-D sim` runs the *real* bmi323 driver, so accel and gyro
  reach the FC by being pushed into the emulated part and read back over emulated SPI. Without
  `--imu-port` the FC logs "IMU stopped producing data" and never gets an attitude.
* **USART3 -> port 4001** — the **RX UART**, carrying real CRSF frames. This is the FC's only RC
  input; without `--rc-port` on the bridge the FC gets no sticks and **will never arm**.

Run the two in separate terminals - `renode` blocks until Ctrl-C, and you often want to restart
`sim` against a still-running emulator. **Restart Renode between runs** rather than re-attaching:
a gap in the sensor stream gives `Nav_Update` one enormous `dt`, which throws the attitude estimate
and costs several seconds of beta-limited recovery.

Requires a **`--single-core`** build (`renode` refuses a dual-core image, which would otherwise
hang silently in `main()` waiting for the CM4). Renode itself lives in `Tools/renode/`; the
platform overlay and machine script are in `Scripts/renode/`. Flags: `--port` (sim link, 4000),
`--rc-port` (CRSF, 4001), `--monitor-port` (3456), `--elf` to override, `--gui` for the Renode
window.

How the rig works, what it measures and where it lies to you: `SilResearch.md`; every firmware
issue it found, and the open ones: `KnownIssues.md`.

**Debugging a GNC bug rather than running a command?** Use the **flapjack-sil-debugging** skill:
getting observability out of a `-D sim` build, isolating firmware faults
from flight-model faults, and sweeping gains without a Renode round trip.

### `build` / `flash` flags (`-f`, combine freely)

`d`=Debug (default), `r`=Release, `c`=clean, `t`=HIL tests, `g`=regenerate proto+umsg.
Driver profile via `-D` on `build` (`default`=real hardware, `sim`=HIL). Artifacts land
in `Build/<board>/<config>/` as `cm7.elf` / `cm4.elf`; a `--single-core` build goes to
`Build/<board>/<config>-single-core/` (`cm7.elf` only) so a stale `cm4.elf` can never sit
beside a single-core image.

### `sim` (host side of the sim link)

Everything after `sim` is forwarded verbatim to the JSBSim bridge. Key flags: `--port`
(required), `--baud` (default **460800**, must equal the board's `SERIAL_LINK_BAUD_RATE`), `--rate`
(Hz, default 400), `--model` (default `tiltrotor`), `--dry-run`.

RC and flight plans:

| Flag | Meaning |
|---|---|
| `--rc-port` | port carrying CRSF to the FC's RX UART. **The only RC path** — omit it and the FC never arms |
| `--rc-baud` | 416666 (the CRSF spec default); ignored for a `socket://` URL |
| `--rc-rate` | CRSF frame rate, default 50 Hz (a real link runs 50-150; the FC polls at 50) |
| `--plan` | YAML flight plan to fly once armed; the bridge exits with a check summary when it ends |
| `--rc-stop N` | stop sending CRSF after N s, imitating a receiver failsafe |
| `--hold-until-armed` | freeze the FDM at its IC until the FC arms (default on; `--no-hold-until-armed` disables) |

With `--plan`, the exit code is the verdict: **0 = checks passed, 1 = failed** (NaN, motor pinned
at 1.000, motor outside [0,1], servo outside ±π/2, or never armed). Full help:
`python Scripts/board.py sim --help`; background: `Scripts/sim/README.md`.

### Flight plans

`Scripts/sim/plans/*.yaml` are scripted stick sequences — `hover`, `yaw_step`, `roll_pitch`.
A plan is a list of timed segments that `set` or `ramp` named channels (`roll`, `pitch`, `yaw`,
`throttle`, `aux1`…) in microseconds; anything unmentioned holds.

Two things make runs comparable, and both are easy to get wrong if you write your own driver:
**arming is a barrier, not a timestamp** (the plan clock starts when the FC reports armed, because
the attitude-settle gate takes a variable amount of time), and **`t` is simulation time**, not wall
clock — the FDM advances exactly one `dt` per tick, so `tick * dt` is exact and drift-free.

## When to use which

- **Changed firmware, want to see it build / on hardware** → `build`, then `flash` (pair
  with the `run` / `verify` skills to confirm behavior).
- **Testing the GNC loop without hardware, or working on `SIM_HIL` / `drivers/sim_link` /
  `Scripts/sim/bridge.py`** → `build -D sim`, `flash`, then `sim --dry-run` FIRST (validates
  serial link, framing, and telemetry echo) before the full `sim`.
- **Edited a `.proto` or umsg def** → `python Scripts/board.py gen` (no toolchain/build needed);
  it regenerates BOTH the firmware nanopb (`Firmware/msgs/proto/*.pb.{c,h}`) and the Python
  stubs in `Scripts/proto/` (the single home — every tool does `from proto import …`).
  `build -f g` does the same as part of a full build.

## Gotchas

- **Baud must match on both ends.** `bridge.py --baud`, the GUI and the board's
  `SERIAL_LINK_BAUD_RATE` (`Firmware/target/<board>/<board>.h`) are all **460800**. One
  number now, for every build — `SIM_LINK_BAUD` is gone.
- **Never hand-edit `*_pb2.py`.** They live only in `Scripts/proto/` and are produced by
  `board.py gen` (or `build -f g`), which generates them with the venv's `grpc_tools` so the
  gencode matches the installed `protobuf` runtime. If you ever hit a protobuf `VersionError`,
  just re-run `board.py gen`.
- **A `-D sim` build shares the debug UART.** Binary sim frames, ASCII debug logs and shell
  commands all ride it together (`Firmware/drivers/serial/serial_link.c`), so logging and the
  shell both work in HIL. `bridge.py` prints the log text to stdout.
- **`--log-level {none,error,warn,info,debug}` compiles logs out**, it does not filter at
  runtime — the strings leave the image entirely (~9.5 kB at `none`). Default is `debug`.
  Careful: `debug` also gates `LOG_DATA`, which is what the GUI plots, so a build at `info` or
  below silently stops those plots. Changing the level forces a clean rebuild, because a `-D`
  change alone does not reliably invalidate objects under MinGW Make.
- **SIL needs `--single-core`.** `SINGLE_CORE` also makes CM7 the primary logger; without it
  nothing drains the log ring buffer and the board is silent.
- **The FC owns the arming interlock, not the bridge.** `tasks/mission/mission.c` refuses to
  arm until the attitude estimate has held still (within 1.0 deg for 3 s) and throttle is at
  minimum, and it treats a raised arm switch as a standing request until its gate opens. `sim`
  just plays pilot: throttle down, switch up after `--arm-delay` (2.0 s), wait for
  `Telemetry.armed`. From cold the estimate converges in about 3 s, so expect to arm ~6 s in.
- **RC reaches the FC as real CRSF, on its own wire.** There is no shortcut into `g_Rx.channels`
  any more — the old `RcInput` sim-link message and its handler are gone, and frame id **2 is
  retired** (do not reuse it). The bridge sends spec-conformant 0x16 frames on `--rc-port`, so a
  SIL run exercises the UART ISR, CRSF deframing, CRC, the 11-bit unpack and the channel mapping
  exactly as flight does. `Scripts/sim/crsf.py` is the encoder; `Tests/UnitTest/test_crsf.c`
  holds a golden frame that both sides are asserted against.
- **Host unit tests need `-mno-ms-bitfields` on MinGW.** `Tests/CMakeLists.txt` sets it. Without
  it, MSVC bitfield layout packs `CrsfChannelsPayload_t` to 32 bytes instead of the 22 ARM EABI
  produces, and the host silently tests a wire format the firmware never emits.
- **Bring the link up with `--dry-run` first** (emits a fixed 20° roll, no JSBSim needed) to
  isolate link/baud/framing problems from the flight model.
