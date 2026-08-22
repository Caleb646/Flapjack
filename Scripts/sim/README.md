# Flapjack JSBSim HIL Simulator

Hardware-in-the-loop sim for the tilt-rotor. The firmware runs on the real
STM32H745 **or on an emulated CM7 under Renode** (see [SIL mode](#sil-mode-no-board-required)
below); this PC tool runs **JSBSim** and exchanges data with it over the (now
binary) debug UART:

```
 PC: JSBSim + bridge.py  ◄──ServoCmd/MotorCmd────  FC (sim driver profile)
                         ◄──Telemetry────────────  (validation + sensor loopback)

                         ──CRSF 0x16────────────►  FC RX UART  (USART3)
                         ──NMEA GGA/RMC─────────►  FC GPS UART (USART2)

                         ──accel, gyro──────────►  emulated BMI323  (tcp 4010) ─► SPI1
                         ──field────────────────►  emulated MMC5983 (tcp 4011) ─┐
                         ──Pa, degC─────────────►  emulated BMP390  (tcp 4012) ─┴► SPI5
```

**Nothing reaches the FC pre-decoded.** RC and GPS go down separate wires as real
CRSF and real NMEA, exactly as the parts would drive them, so a SIL run exercises
the UART ISRs, the deframing, the checksums and the parsers. There used to be an
`RcInput` sim-link message that wrote `g_Rx.channels` directly; it bypassed all
of that and has been removed.

The three **sensors** follow the same principle one level down. A register read
is not a byte protocol, but it is still a path with a chip select, a dummy byte,
an address auto-increment and a data-ready bit to get wrong — so rather than
inject decoded values at the driver, the bridge pushes physical quantities into
emulated parts and the firmware reads them back through its real `bmi323`,
`mmc5983` and `bmp390` drivers over emulated SPI. That put five firmware defects
on the record for the IMU alone (KnownIssues §2.29–§2.33), and for the baro it
puts the 21-byte calibration decode and the compensation polynomial on the
critical path.

**Frame ids 1 (`SensorData`), 2 (`RcInput`) and 7 (`BaroData`) are retired — do
not reuse them.** The sim link is FC→PC only now, apart from the shell.

- **PC → FC:** IMU (accel m/s², gyro deg/s) and mag (**gauss**) pushed at 400 Hz,
  baro (Pa, °C) at 50 Hz, each into its own emulated part; RC channels as CRSF
  `0x16` frames on the RX UART; GPS as NMEA `GGA`/`RMC` on the GPS UART.
- **FC → PC:** per-servo tilt **angle** (rad) and per-motor **throttle** (0–1),
  plus a `Telemetry` frame (nav euler, armed, IMU sample count, RC link state,
  and the baro/GPS **loopback** described below).

Mag is pushed in gauss, not as a unit vector, because the MMC5983 has a real
±8 G full scale that `mmc5983.c` divides by — a unit vector would assert that
Earth's field saturates the part. Nothing downstream depends on the magnitude
(Madgwick renormalises), but the scale is now a testable property rather than an
assumption.

### The sensor loopback

`Telemetry` echoes back the baro pressure and GPS fix the firmware decoded, read
from the **umsg topics** rather than from the sim link's own decode slots. The
bridge asserts every echo matches something it actually sent:

- **baro: bit-exact.** Both hops are IEEE 754 binary32, so a correct chain
  reproduces the sent value exactly — no tolerance, and under `--dry-run` the
  pressure is *swept* so a latched or stale value cannot pass by accident.
- **GPS: within ~1.1 cm.** NMEA quantises the coordinate to 5 decimal places of a
  minute (~1.9 cm), so the check compares against the post-quantisation value.
  The tolerance is tight enough that narrowing the chain back to `float`
  (~2.2e-6 deg at mid latitudes) fails.

This covers the whole chain — wire, driver, device, umsg publish — in one
comparison. It is specifically the check that catches a parser which succeeds
while discarding its result, which is exactly what `Gps_Update_` used to do.

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
for imu/mag/baro/servo/motor and defines `SIM_HIL`):

```
python Scripts/board.py build -b flapjack-v1 -D sim
```

In this build the shared UART (UART_1, `SERIAL_LINK_BAUD_RATE` = 460800) carries binary sim
frames **and** ASCII debug logs, interleaved; the shell shares it too, as frame id
6. `bridge.py` separates them and prints the log text to stdout. Log output is
7-bit ASCII and the frame magic is `0xAA`, which is what makes the two
unambiguous — see `Firmware/drivers/serial/serial_link.h`.

Wire the board's UART_1 TX/RX to a USB-serial adapter on the PC. On real hardware
the receiver drives UART_3 (`RX_UART`, 416666 baud 8N1) as usual — the bridge's
`--rc-port` stands in for it only under Renode.

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

The bridge loads `Scripts/sim/jsbsim/aircraft/tiltrotor/tiltrotor.xml`, raises the
arm switch after `--arm-delay`, waits for the FC to report armed, then goes to
hover throttle, applies the FC's servo/motor commands to the model, and streams
synthesized sensors back.

**Every wire defaults to its SIL socket**, so a SIL run is just `board.py sim` with no flags.
The defaults apply only while `--port` is a socket, i.e. while this is a SIL run — naming a real
device turns them all off, because on a board the companion wires are physical ports nobody can
guess. Pass `none` to any one of them (`--gps-port none`) to leave that wire off, which is how
the "no receiver" and "no fix" paths stay testable. Useful flags:

| flag | default | meaning |
|---|---|---|
| `--port` | `socket://localhost:4000` | the FC's sim link. The default is a SIL run; give a real device (`COM7`, `/dev/ttyUSB0`) to drive a board |
| `--baud` | 460800 | must match the board's `SERIAL_LINK_BAUD_RATE` |
| `--rate` | 400 | sensor stream rate (Hz) |
| `--model` | tiltrotor | JSBSim aircraft name under `--root/aircraft/` |
| `--hover-throttle` | 0.5 | throttle held once armed, when no `--plan` is given |
| `--rc-port` | `socket://localhost:4001` | port carrying CRSF to the FC's RX UART. **The only RC path** — with `none` the FC never arms |
| `--rc-baud` | 416666 | CRSF spec default; ignored for a `socket://` URL |
| `--rc-rate` | 50 | CRSF frame rate (Hz); a real link runs 50–150 |
| `--imu-port` | `socket://localhost:4010` | socket carrying accel/gyro to the emulated BMI323. **The only IMU path** — with `none` there is no attitude |
| `--mag-port` | `socket://localhost:4011` | socket carrying the field in gauss to the emulated MMC5983. **The only mag path** |
| `--baro-port` | `socket://localhost:4012` | socket carrying Pa/°C to the emulated BMP390. **The only baro path** |
| `--baro-rate` | 50 | barometer push rate (Hz); 0 disables baro |
| `--gps-port` | `socket://localhost:4002` | port carrying NMEA to the FC's GPS UART. **The only GPS path** |
| `--gps-baud` | 115200 | matches `GPS_BAUD_RATE`; ignored for a `socket://` URL |
| `--gps-rate` | 10 | fix rate (Hz). GGA and RMC go out half a period apart — the driver holds one sentence, so a burst would lose the first |
| `--plan` | (none) | YAML flight plan to fly once armed |
| `--rc-stop` | (none) | stop sending CRSF after N seconds, imitating a receiver failsafe |
| `--hold-until-armed` | on | freeze the FDM at its IC until the FC arms |

### Flight plans

`--plan Scripts/sim/plans/hover.yaml` replays a scripted stick sequence instead
of the fixed arm-and-hover gesture. A plan is a list of timed segments that `set`
or `ramp` named channels in microseconds:

```yaml
name: yaw_step
plan:
  - {t: 0.0,  dur: 3.0}
  - {t: 3.0,  dur: 2.0, ramp: {throttle: 1500}}
  - {t: 8.0,  dur: 2.0, set:  {yaw: 1700}}
  - {t: 10.0, dur: 3.0, set:  {yaw: 1500}}
```

Two properties make runs comparable. **Arming is a barrier, not a timestamp** —
the plan clock starts when the FC reports armed, because the attitude-settle gate
takes a variable amount of time (measured anywhere from 3 s to ~24 s). And **`t`
is simulation time**: the FDM advances exactly one `dt` per tick, so `tick * dt`
is exact, where wall-clock pacing would move the sticks around under host load.

When the plan ends the bridge prints a check summary and exits **0 (pass) or
1 (fail)**. The checks are physics invariants — no NaN, motors inside [0,1] and
not pinned at 1.000, servos inside ±π/2, FC actually armed — deliberately *not* a
golden trajectory, because manual mode is a rate loop and the attitude a run
settles at is not repeatable (`KnownIssues.md` §1.13).

The **sensor loopback** lines are the exception: they *are* exact assertions, and
they can be, because they compare the firmware against what the bridge itself
sent rather than against an unvalidated flight model. A healthy run reads:

```
[checks] baro         2342 sent, 2331 echoed, 0 mismatched
[checks] gps          976 sent, 976 echoed, 0 mismatched
```

Both lines are silent when the sensor was not streamed, so an older plan or a
`--baro-rate 0` run still passes. `0 echoed` against a non-zero `sent` is a
failure, not a skip — that is the "FC decoded none" case.

## Wire protocol

`[0xAA][0x55][msg_id][len][payload][crc8]` — `crc8` poly 0x07, init 0x00, over
`(msg_id,len,payload)`; payload is protobuf (`sim.proto`). Defined identically in
`Firmware/drivers/sim_link/sim_link.c` and the shared host framing
`Scripts/link/framing.py` (used by the bridge).

## Status / caveats

- **The CRSF path is unit-tested on the host.** `Tests/UnitTest/test_crsf.c` checks
  the firmware's deframer, CRC, channel unpack and tick→µs mapping against the spec,
  and asserts a golden frame produced by `Scripts/sim/crsf.py` byte for byte, so the
  encoder here and the decoder there cannot drift apart.
- **The NMEA path is too.** `Tests/UnitTest/test_gps.c` does the same job for GPS
  against golden sentences from `Scripts/sim/nmea.py`: the byte assembler
  (resync, partial sentences, one-byte-at-a-time delivery), the coordinate and
  knots→m/s conversions, checksum rejection, and that a void RMC or a
  `fix_quality` 0 GGA is **not** reported as a fix.
- **The sensor models come from JSBSim.** `SensorBaro.xml` and `SensorGps.xml` are
  vendored into `jsbsim/systems/` from the JSBSim distribution and included by
  `tiltrotor.xml`. Their `<noise>`/`<bias>`/`<drift_rate>` terms are **on** — see
  `KnownIssues.md` §3.13 for each value and why. Tune realism **there**, not in
  `bridge.py`. There is still **no IMU noise model**: the gyro and accel the
  bridge synthesises are exact. `SensorImu.xml` is deliberately **not** used —
  its accelerometer reports the opposite Z sign to this project's convention, and
  its magnetometer disagrees with WMM in both sign and magnitude (32.2 µT against
  ~48 µT real).
- **The FDM is sized to the real airframe** (0.85 kg, 7 in props, 1000 KV assumed
  on 6S): hover at 50 % throttle, 2:1 thrust-to-weight, tilt verified against
  measured force directions. Thrust comes from `<external_reactions>` rather than
  JSBSim's propeller model, which diverged — see `SilResearch.md` §6.
  Inertias are still estimates, and `ROTOR_TMAX` (1.874 lbf per rotor) is the one
  number to replace with thrust-stand data.
- **Forward flight is approximated** by a thrust-vs-airspeed table rather than
  blade-element physics. Fine for hover and low-speed control work; revisit before
  trusting cruise or transition results.
- **The gains are provisional.** The stack flies closed-loop, but the tune is
  fitted to an estimated rotor height and unmeasured servo dynamics — see
  `KnownIssues.md` §1.9. `roll_pitch.yaml` still departs (§1.14); `hover`,
  `yaw_step` and `alt_hold` pass.

---

## SIL mode (no board required)

Renode emulates the CM7 and exposes **both** UARTs as TCP servers, so the bridge
talks to sockets instead of COM ports — USART1 (sim link) on 4000 and USART3
(the RX UART, carrying CRSF) on 4001. Nothing else changes: same framing, same
protobuf messages, same `bridge.py`.

```bash
python Scripts/board.py build -b flapjack-v1 -D sim --single-core
python Scripts/board.py renode -b flapjack-v1                          # terminal 1
python Scripts/board.py sim                                            # terminal 2
```

USART2 (the GPS UART) is exposed on **4002** alongside the other two. Like
USART3 it needed no Renode work beyond a frequency pin — it is a real
`UART.STM32F7_USART` in the shipped platform. `--gps-port` is optional: leave it
off and `Gps_Task` simply never sees a fix, and the GPS checks stay silent.

`renode` takes the place of `flash`; it blocks until Ctrl-C. **Restart it between
runs** rather than re-attaching the bridge: a gap in the sensor stream hands
`Nav_Update` one enormous `dt`, which throws the attitude estimate and costs
several seconds of recovery.

Measured behaviour (see `SilResearch.md` §3):

- Renode tracks wall clock **1.00x** under load, so the bridge's real-time pacing
  is correct as-is.
- **400 Hz sustains with 0.0 % dropped `SensorData`**; the ceiling is ~740/s,
  beyond which throughput collapses.
- `Telemetry.imu_count` vs frames sent is the health metric — it should track 1:1.

The FC arms itself: `bridge.py` holds throttle low with the switch up after
`--arm-delay` (2.0 s) and waits for `Telemetry.armed`. The interlock that decides
when is in the firmware, not the bridge — the estimate must be converged and
still for 3 s (`KnownIssues.md` §2.13) — so arming lands anywhere from ~3 s to
~24 s in. Ground start and landing are fine; the gear no longer diverges (§3.4).
