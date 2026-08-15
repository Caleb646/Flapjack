# Flapjack JSBSim HIL Simulator

Hardware-in-the-loop sim for the tilt-rotor. The firmware runs on the real
STM32H745 **or on an emulated CM7 under Renode** (see [SIL mode](#sil-mode-no-board-required)
below); this PC tool runs **JSBSim** and exchanges data with it over the (now
binary) debug UART:

```
 PC: JSBSim + bridge.py  ──SensorData/RcInput──►  FC (sim driver profile)
                         ◄──ServoCmd/MotorCmd────  Control loop
                         ◄──Telemetry────────────  (validation)
```

- **PC → FC:** synthesized IMU (accel m/s², gyro deg/s) + mag (normalized), and
  an injected RC frame (armed + hover throttle).
- **FC → PC:** per-servo tilt **angle** (rad) and per-motor **throttle** (0–1),
  plus a `Telemetry` frame (nav euler, armed, IMU sample count) for validation.

Timing is **free-running real-time**: sensors stream at a fixed rate and JSBSim
is stepped to wall-clock; the FC consumes the latest sample and replies async.

## 1. Install

From the repo root (installs JSBSim + the shared base deps):

```
pip install -e ".[sim]"
```

The Python proto stub lives at `Scripts/proto/sim_pb2.py` and is regenerated
(alongside the firmware's nanopb `Firmware/msgs/proto/sim.pb.{c,h}`) by:

```
python Scripts/board.py gen
```

(`build -b flapjack-v1 -D sim -f g` regenerates the same as part of a full build.)
`sim.proto` is the **single source of truth** for both.

## 2. Build + flash the HIL firmware

Configure the firmware with the `sim` driver profile (selects the sim backends
for imu/mag/servo/motor and defines `SIM_HIL`):

```
python Scripts/board.py build -b flapjack-v1 -D sim
```

In this build the debug UART (UART_1, `SIM_LINK_BAUD` = 460800) carries **binary
sim frames only** — logging/shell are disabled. Logs are unavailable on UART
while in HIL (use SWO if you need them).

Wire the board's UART_1 TX/RX to a USB-serial adapter on the PC.

## 3. Bring up the link FIRST (no FDM needed)

`--dry-run` skips JSBSim and emits a fixed **20° roll** attitude. This validates
the serial link, the FC's sensor decode/frames, and the telemetry echo without
any flight model:

```
python Scripts/board.py sim --port COM7 --dry-run
```

Expected: the `[FC] euler(deg)=` line should converge to roll ≈ **+20°**,
`armed=True` shortly after start, and `imu#` increasing (proves the IMU task is
paced by the incoming stream). If euler roll is negative or on the wrong axis,
flip the sign in `synthesize_sensors()` (clearly marked).

## 4. Run the full sim

```
python Scripts/board.py sim --port COM7
```

The bridge loads `Scripts/sim/jsbsim/aircraft/tiltrotor/tiltrotor.xml`, sends arm + hover
throttle ~1 s after start, applies the FC's servo/motor commands to the model,
and streams synthesized sensors back. Useful flags:

| flag | default | meaning |
|---|---|---|
| `--port` | (required) | serial port, e.g. `COM7`, `/dev/ttyUSB0` |
| `--baud` | 460800 | must match `SIM_LINK_BAUD` |
| `--rate` | 400 | sensor stream rate (Hz) |
| `--model` | tiltrotor | JSBSim aircraft name under `--root/aircraft/` |
| `--hover-throttle` | 0.5 | injected throttle once armed |

## Wire protocol

`[0xAA][0x55][msg_id][len][payload][crc8]` — `crc8` poly 0x07, init 0x00, over
`(msg_id,len,payload)`; payload is protobuf (`sim.proto`). Defined identically in
`Firmware/drivers/sim_link/sim_link.c` and the shared host framing
`Scripts/link/framing.py` (used by the bridge).

## Status / caveats

- **`bridge.py` protocol + sensor-synthesis math are unit-tested** (CRC, framing,
  resync, bad-CRC rejection, level/tilt synthesis, RC). The CRC matches the FC.
- **The FDM is sized to the real airframe** (0.85 kg, 7 in props, 1000 KV assumed
  on 6S): hover at 50 % throttle, 2:1 thrust-to-weight, tilt verified against
  measured force directions. Thrust comes from `<external_reactions>` rather than
  JSBSim's propeller model, which diverged — see `EmulatorResearch.md` §13.
  Inertias are still estimates, and `ROTOR_TMAX` (1.874 lbf per rotor) is the one
  number to replace with thrust-stand data.
- **Forward flight is approximated** by a thrust-vs-airspeed table rather than
  blade-element physics. Fine for hover and low-speed control work; revisit before
  trusting cruise or transition results.
- Achieving a stable hover still needs the tilt-rotor **mixer + hover control
  law**. The SIL has already caught two faults there: motor output saturates at
  1.000 with the throttle stick at minimum, and servo output is unclamped
  (~41,553 µs against a 500–2500 µs range).

---

## SIL mode (no board required)

Renode emulates the CM7 and exposes USART1 as a TCP server, so the bridge talks
to `socket://localhost:4000` instead of a COM port. Nothing else changes — the
same framing, the same protobuf messages, the same `bridge.py`.

```bash
python Scripts/board.py build -b flapjack-v1 -D sim --single-core
python Scripts/board.py renode -b flapjack-v1                          # terminal 1
python Scripts/board.py sim --port socket://localhost:4000 --rate 400  # terminal 2
```

`renode` takes the place of `flash`; it blocks until Ctrl-C. Keep it in its own
terminal so you can restart the bridge without rebooting the emulator.

Measured behaviour (see `EmulatorResearch.md` §12):

- Renode tracks wall clock **1.00x** under load, so the bridge's real-time pacing
  is correct as-is.
- **400 Hz sustains with 0.0 % dropped `SensorData`**; the ceiling is ~740/s,
  beyond which throughput collapses.
- `Telemetry.imu_count` vs frames sent is the health metric — it should track 1:1.

Two things to know before running closed-loop (without `--dry-run`):

- The attitude filter takes **tens of seconds** to converge from cold. The
  `--arm-delay` default of 1.0 s arms into a garbage estimate and tumbles the
  aircraft; hold level and raise it.
- The FDM NaNs on hard ground impact, so start airborne for now.
