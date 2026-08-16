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
| Bring up the sim link (no FDM) | `python Scripts/board.py sim --port <PORT> --dry-run` |
| Run the full JSBSim sim | `python Scripts/board.py sim --port <PORT>` |
| Launch the flight GUI | `python Scripts/board.py gui` |
| Build for the Renode SIL | `python Scripts/board.py build -b flapjack-v1 -D sim --single-core` |
| Boot the firmware in Renode | `python Scripts/board.py renode -b flapjack-v1` |
| Drive the SIL from the bridge | `python Scripts/board.py sim --port socket://localhost:4000 --rate 400` |

Boards: `flapjack-v1`, `nucleo-h747zi`.

### SIL (software-in-the-loop) via Renode

`renode` stands in for `flash`: it boots `cm7.elf` on an emulated Cortex-M7 and exposes USART1
as a TCP server, so `sim` connects to `socket://localhost:4000` instead of a COM port. Nothing
else about the bridge changes. Run the two in separate terminals - `renode` blocks until Ctrl-C,
and you often want to restart `sim` against a still-running emulator.

Requires a **`--single-core`** build (`renode` refuses a dual-core image, which would otherwise
hang silently in `main()` waiting for the CM4). Renode itself lives in `Tools/renode/`; the
platform overlay and machine script are in `Scripts/renode/`. Flags: `--port` (sim link, 4000),
`--monitor-port` (3456), `--elf` to override, `--gui` for the Renode window.

Full background, including every emulator/firmware issue found and fixed: `EmulatorResearch.md`;
current state and open items: `KnownIssues.md`.

**Debugging a GNC bug rather than running a command?** Use the **flapjack-sil-debugging** skill:
getting observability out of a `-D sim` build (which has no logging), isolating firmware faults
from flight-model faults, and sweeping gains without a Renode round trip.

### `build` / `flash` flags (`-f`, combine freely)

`d`=Debug (default), `r`=Release, `c`=clean, `t`=HIL tests, `g`=regenerate proto+umsg.
Driver profile via `-D` on `build` (`default`=real hardware, `sim`=HIL). Artifacts land
in `Build/<board>/<config>/` as `cm7.elf` / `cm4.elf`; a `--single-core` build goes to
`Build/<board>/<config>-single-core/` (`cm7.elf` only) so a stale `cm4.elf` can never sit
beside a single-core image.

### `sim` (host side of the sim link)

Everything after `sim` is forwarded verbatim to the JSBSim bridge. Key flags: `--port`
(required), `--baud` (default **460800**, must equal firmware `SIM_LINK_BAUD`), `--rate`
(Hz, default 400), `--model` (default `tiltrotor`), `--dry-run`. Full help:
`python Scripts/board.py sim --help`; background: `Scripts/sim/README.md`.

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

- **Sim baud must match on both ends.** `bridge.py --baud` and firmware `SIM_LINK_BAUD`
  (`Firmware/drivers/sim_link/sim_link.h`) are both **460800** — change them together.
- **Never hand-edit `*_pb2.py`.** They live only in `Scripts/proto/` and are produced by
  `board.py gen` (or `build -f g`), which generates them with the venv's `grpc_tools` so the
  gencode matches the installed `protobuf` runtime. If you ever hit a protobuf `VersionError`,
  just re-run `board.py gen`.
- **A `-D sim` build takes over the debug UART** for binary sim frames — logging and the
  shell are disabled in HIL (use SWO for logs).
- **SIL needs `--single-core`.** `SINGLE_CORE` also makes CM7 the primary logger; without it
  nothing drains the log ring buffer and the board is silent.
- **The FC owns the arming interlock, not the bridge.** `tasks/mission/mission.c` refuses to
  arm until the attitude estimate has held still (within 1.0 deg for 3 s) and throttle is at
  minimum, and it treats a raised arm switch as a standing request until its gate opens. `sim`
  just plays pilot: throttle down, switch up after `--arm-delay` (2.0 s), wait for
  `Telemetry.armed`. From cold the estimate converges in about 3 s, so expect to arm ~6 s in.
- **Bring the link up with `--dry-run` first** (emits a fixed 20° roll, no JSBSim needed) to
  isolate link/baud/framing problems from the flight model.
