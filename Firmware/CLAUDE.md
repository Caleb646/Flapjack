# Firmware Architecture — GNC Layers

This file is loaded automatically by Claude Code for any work under `Firmware/`. It documents the pub/sub GNC architecture so that agents implementing FreeRTOS tasks know exactly how the layers are structured.

---

## GNC Architecture

```
Sensors ──► Navigation ──► Guidance ──► Control
               ▲               ▲
             Mission ──────────┘
```

Each layer has one job, publishes one message type, and only subscribes to layers below it. No layer reads from a layer above it.

---

## Layer Reference

Every cross-layer dependency is a **subscription**. A layer's primary input is a blocking
`receive()`; every additional input is a non-blocking `receive(..., 0)` whose result is cached
locally. There is no way to read a message without subscribing to it.

Paths are relative to `Firmware/`. Each task lives in its own folder under `tasks/`;
the device and driver layers it uses sit in `devices/` and `drivers/`.

| Layer | Task source | Init fn | Update fn | Blocks on | Also subscribes (non-blocking, cached) | Publishes |
|---|---|---|---|---|---|---|
| **Sensor: IMU** | `tasks/imu/imu_task.c` (device `devices/imu.c`) | `Imu_Init()` | `Imu_Update()` | hardware (sim link / ISR) | - | `umsg_sensors_imu_t` |
| **Sensor: Mag** | `tasks/mag/mag_task.c` (device `devices/mag.c`) | `Mag_Init()` | `Mag_Update()` | hardware | - | `umsg_sensors_mag_t` |
| **Sensor: Baro** | `tasks/baro/baro_task.c` (device `devices/baro.c`) | `Baro_Init()` | `Baro_Update()` | hardware (sim link `BaroData`) | - | `umsg_sensors_baro_t` |
| **Sensor: GPS** | `tasks/gps/gps_task.c` (device `devices/gps.c`) | `Gps_Init()` | `Gps_Update()` | nothing - 100 Hz `vTaskDelay` | - | `umsg_sensors_gps_t` |
| **Sensor: RC** | `tasks/rc/rc.c` (driver `drivers/rx/rx.c` + `crsf.c`) | `Rc_Init()` | `Rc_Update()` | nothing - 50 Hz `vTaskDelay` | - | `umsg_rc_input_t` |
| **Navigation** | `tasks/nav/nav.c` | `Nav_Init()` | `Nav_Update()` | `umsg_sensors_imu_t` | `umsg_sensors_mag_t`, `umsg_sensors_baro_t`, `umsg_sensors_gps_t` | `umsg_nav_state_t` |
| **Mission** | `tasks/mission/mission.c` | `Mission_Init()` | `Mission_Update()` | `umsg_rc_input_t` (20 ms timeout) | `umsg_arming_request_t`, `umsg_nav_state_t` | `umsg_mission_state_t` |
| **Guidance** | `tasks/guidance/guidance.c` | `Guidance_Init()` | `Guidance_Update()` | `umsg_nav_state_t` | `umsg_mission_state_t`, `umsg_rc_input_t` | `umsg_guidance_setpoints_t` |
| **Control** | `tasks/control/control.c` (mixer `tasks/control/mixer.c`) | `Control_Init()` | `Control_Update()` | `umsg_guidance_setpoints_t` | `umsg_nav_state_t`, `umsg_mission_state_t`, `umsg_tune_pid_t` | motors/servos (direct) |

Tasks are created and registered in `Firmware/main.c`.

**Baro and GPS are consumed by `Nav_Update`.** Baro drives a vertical complementary filter and
GPS drives the horizontal position/velocity estimate — see "The altitude and position estimate"
below.

### The RC path

A CRSF receiver drives the RX UART (`RX_UART`, 416666 baud 8N1). The byte-wise ISR in
`drivers/rx/crsf.c` reassembles frames; `Rc_Task` polls `Rx_Update()` at 50 Hz to validate and
decode them, then publishes `umsg_rc_input_t`. Same shape as `Imu_Task` / `Mag_Task`: one task, its
device underneath, one topic out. `g_Rx` is internal to the driver — reach it through
`Rx_GetChannels()` / `Rx_IsLinkUp()`, not directly.

**`Rc_Update` publishes on its own clock, not on frame arrival.** Most polls find no completed
frame, and that is the ordinary case rather than an error. Publishing regardless is what lets a lost
link be *reported* (`link_quality` drops to 0) instead of the chain simply going quiet — a dead link
produces no frames at all, so absence of data cannot itself carry the signal. Link liveness comes
from frame *timing* in the driver (`Rx_IsLinkUp()`, `RX_LINK_TIMEOUT_US`).

This used to be two tasks with a `umsg_rx_channels_t` topic between them. That split was a
core-allocation artifact — the decode half ran on CM4 and reached CM7 through shared memory — and
once both moved to CM7 it only added a second 20 ms polling hop, doubling stick-to-guidance latency
for no benefit. The topic had exactly one subscriber.

**Everything runs on CM7; CM4 has no application tasks.** Worth knowing before putting one there:
umsg is plain FreeRTOS queues with per-core `static` topic metadata, so a message published on CM4
is invisible to a CM7 subscriber. Crossing cores needs `core/sync.c`, not umsg.

This matters for the SIL: RC is fed in as **real CRSF frames on the RX UART**, not injected
anywhere downstream, so the emulated run exercises the whole chain. The channel scaling follows the
CRSF spec exactly — `TICKS_TO_US(x) = (x - 992) * 5 / 8 + 1500`, so ticks 192/992/1792 are
1000/1500/2000 µs. Getting that mapping wrong puts a standing rate demand on all three axes with
the sticks centred, which is what `CRSF_CHANNEL_MIN/MAX` being wrong used to do.

The SIL-only `SimTelemetry_Task` (`tasks/sim/sim_telemetry.c`) subscribes to `umsg_nav_state_t`
and `umsg_mission_state_t` the same way; it blocks on nothing and runs off `vTaskDelay()` at 50 Hz.
It also subscribes to `umsg_sensors_baro_t` and `umsg_sensors_gps_t` and echoes them back to the
bridge, which asserts that what it sent is what the firmware decoded — see "The sensor loopback".

### The GPS path

Mirrors the RC path exactly. A receiver drives the GPS UART (`GPS_UART`, 115200 baud 8N1) with
NMEA; the byte-wise ISR in `drivers/gps/gps.c` assembles sentences, and `Gps_Task` polls
`Gps_Update()` at 100 Hz to parse them with `minmea`, and publishes `umsg_sensors_gps_t` on
any sentence that carried a position.

**Only a sentence carrying an actual fix publishes.** A void RMC (status `V`), a GGA with
`fix_quality` 0, and the non-positional types (GSV/GST) all return non-`SUCCESS`, so a receiver
with no lock cannot be mistaken for one sitting at 0,0. This matters more than it looks: the
parser used to run and then discard every field while returning `SUCCESS`, so a fix and no fix
were indistinguishable forever.

**The driver holds exactly one assembled sentence.** A second landing before `Gps_Task` polls
overwrites the first, so the poll rate has to outpace the *sentence* rate (a 10 Hz receiver
emitting GGA + RMC is 20 sentences/s), and the SIL bridge spaces the two halves of a fix half a
period apart. Renode does not pace bytes at the baud rate, so a back-to-back pair would otherwise
land together.

As with RC, the SIL feeds **real NMEA on the real UART** rather than injecting a decoded fix, so
the emulated run exercises the assembler, the checksum and minmea. `Tests/UnitTest/test_gps.c`
asserts the firmware's parse against golden sentences from `Scripts/sim/nmea.py`, the same encoder
the bridge uses, so host and firmware cannot drift apart.

### The altitude and position estimate

`Nav_Update` blocks on the IMU and does a non-blocking cached `receive` for mag, baro and GPS —
the same pattern for all three secondary inputs. What it does with them differs:

**Vertical (baro + accel).** A third-order complementary filter in `common/filter.c`
(`AltitudeFilter_t`) fusing baro-derived altitude with the vertical acceleration recovered from
the IMU. `Predict` runs at the 400 Hz IMU rate; `Correct` runs only on iterations where a NEW baro
sample arrived, which is why the two are separate calls — re-applying one measurement at 400 Hz
instead of 50 would multiply the effective gain eightfold. The third state is the accelerometer's
vertical bias, without which a constant offset becomes a permanent phantom climb rate.

`Nav_VerticalAccelUp()` owns the frame rotation. It applies only the "down" row of the body→NED
matrix to the specific-force vector and subtracts g. Note the project's accel convention is
`g - a` (level and still reads `(0, 0, +9.81)` in FRD), the negative of physical proper
acceleration, so no extra sign flip is needed — see the comment on that function before touching it.

**The pressure datum is taken at BOOT**, averaged over the first 50 baro samples (~1 s), so
`nav.alt` is height above wherever the board was switched on. Arming would be the better physical
reference, but nav would have to subscribe to `umsg_mission_state_t` and mission already subscribes
to `umsg_nav_state_t` — that closes a dependency cycle the layer rules forbid.

Referencing the sea-level ISA formula to a datum that is not at sea level leaves a systematic scale
error of about **+0.23 % of height per 1000 m of datum elevation** (zero at sea level). Accepted,
because every consumer is relative; `Tests/UnitTest/test_altitude.c` pins the size.

**Horizontal (GPS).** Flat-earth projection of lat/lon onto NED about an origin captured at the
first usable fix, with velocity resolved from the reported speed and course. No accel aiding, so
these step at the receiver's ~10 Hz and hold in between, while the vertical channel next door runs
at 400 Hz. A position-hold loop will want the smooth version; that is a separate job.

**Validity bits do not partition the vectors the way their names suggest**, because the sensors do
not either — baro owns the vertical and GPS owns the horizontal:

| Bit | Covers |
|---|---|
| `NAV_VALID_BARO_ALT` | `alt`, `pos_ned[2]`, `vel_ned[2]` |
| `NAV_VALID_POSITION` | `pos_ned[0..1]` |
| `NAV_VALID_VELOCITY` | `vel_ned[0..1]` |

`Gps_Task` republishes with `fix_type` 0 on a lost or timed-out fix, so nav needs no timer of its
own; the NED origin is deliberately NOT re-derived after a dropout.

### The sensor loopback

Baro and GPS are echoed back in the SIL `Telemetry` frame (`baro_pa`, `gps_lat`, `gps_lon`,
`gps_sats`, plus `*_count` pacing counters). The bridge asserts that every echo matches something
it actually sent — bit-exact for baro (both hops are binary32), within ~1.1 cm for GPS (NMEA
quantises to 5 decimal places of a minute). A run fails if the FC reports a value that was never
sent, or decodes nothing at all.

The echo is read from the **umsg topics**, not from the sim link's own decode slots. That is the
point: it covers the driver, the device and the publish, so a parser that decodes a frame and then
throws the fields away still fails the check.

The `Telemetry` frame also carries `nav_pos_ned`, `nav_vel_ned` and `nav_valid`, which are the
**opposite** of a loopback — nothing sent them to the FC. The bridge compares them against the
FDM's own truth and fails the run if altitude, climb rate or horizontal position drifts past
`NAV_*_EPS` in `bridge.py`, or if a sensor was streamed and its validity bit never appeared.
Measured against the hover plan the estimate tracks truth to 0.01 m and 0.05 m/s.

Adding those fields took `Telemetry_size` to 107 bytes, which is what raised
`SERIAL_LINK_MAX_PAYLOAD` from 96 to 128.

---

## FreeRTOS Task Pattern

Every layer's `_Update` function blocks internally on `portMAX_DELAY` on its primary input subscription. The task body is therefore trivial — no `vTaskDelay` needed.

```c
void nav_task(void* args) {
    Nav_Init();
    while (1) {
        Nav_Update();   // blocks until IMU publishes
    }
}
```

```c
void sensor_imu_task(void* args) {
    while (1) {
        SensorImu_Update();   // blocks on hardware ready (ISR notification or poll)
    }
}
```

### Secondary inputs: subscribe with a local cache

A layer that needs a message it does not block on subscribes with **length 1** — the FreeRTOS port
uses `xQueueOverwrite` for length-1 queues, so the queue always holds the latest value — and then
does a `receive(..., 0)` each iteration. `receive()` **consumes**, so the value must be cached in
the layer's static state; otherwise the layer sees the message only on the iteration it arrived and
garbage (or a stale default) on every other one. This matters because secondary inputs are usually
slower than the loop: RC publishes at 50 Hz while Guidance runs at nav rate (400 Hz+).

```c
// in Xxx_Init()
s_Xxx.rc_sub = umsg_rc_input_subscribe(1, 1);

// in Xxx_Update(), each iteration
umsg_rc_input_receive(s_Xxx.rc_sub, &s_Xxx.rc, 0);   // keeps last value if nothing new
```

Where the consumer must distinguish "never received" from "received earlier", pair the cache with a
`have*` flag (see `Mission_t.haveRc` / `Mission_t.haveNav`) rather than testing the return value of
a single `receive()` — arming must not depend on a message landing in that exact iteration.

There is no `peek()`. `umsg_publish()` does not retain the caller's pointer: it copies into each
subscriber queue and returns, so published buffers do not need static lifetime.

The natural task rates emerge from the blocking receive:
- **IMU task** → drives Nav at IMU ODR (400–800 Hz)
- **Nav task** → drives Guidance at IMU rate
- **Guidance task** → drives Control at nav rate
- **Mission task** → driven by RC link rate (~150 Hz)

Sensor tasks for Mag, GPS, Baro run at their own hardware rates independently — mag off the sim
link's `SensorData`, baro off its own `BaroData` frame (~50 Hz), GPS off the NMEA UART (100 Hz
poll of a ~10 Hz receiver).

---

## umsg Message Types

Generated message structs are in `Firmware/msgs/umsg/`. The JSON definitions are the source of truth at `Firmware/msgs/umsg/defs/`.

| Header | Struct | Key fields |
|---|---|---|
| `umsg_sensors.h` | `umsg_sensors_imu_t` | `gyro[3]` (deg/s), `accel[3]` (m/s²), `temperature` |
| `umsg_sensors.h` | `umsg_sensors_mag_t` | `field[3]` (normalised) |
| `umsg_sensors.h` | `umsg_sensors_baro_t` | `pressure`, `temperature` |
| `umsg_sensors.h` | `umsg_sensors_gps_t` | `lat`, `lon` (double, degrees), `alt`, `speed`, `course`, `fix_type`, `sats` |
| `umsg_rc.h` | `umsg_rc_input_t` | `channels[16]` (µs, 1000–2000), `rssi` (always 0 — needs CRSF 0x14), `link_quality` (100 or 0, from `link_up`) |
| `umsg_nav.h` | `umsg_nav_state_t` | `quat[4]`, `euler[3]` (deg), `gyro[3]` (deg/s), `pos_ned[3]`, `vel_ned[3]`, `alt`, `valid` |
| `umsg_mission.h` | `umsg_mission_state_t` | `mode` (`eMissionMode_t`), `target_pos[3]`, `target_heading`, `armed` |
| `umsg_guidance.h` | `umsg_guidance_setpoints_t` | `w[3]` (rad/s), `vel_b[3]` (body vel, `[2]`=throttle 0–1), `quat[4]` |

### Unit conventions

- **Gyro rates in `umsg_sensors_imu_t` and `umsg_nav_state_t`**: degrees/second — matches the IMU driver and Madgwick filter input.
- **Guidance `w[]`**: radians/second — `control.c` converts to deg/s via `RAD2DEG()` before the rate PID.

### NAV_VALID bits (`umsg_nav_state_t.valid`)

Generated into `msgs/umsg/umsg_nav.h` from the `valid` enum in
`msgs/umsg/defs/nav.json` — edit the JSON and regenerate, not the header:

| Bit | Macro | Meaning |
|---|---|---|
| 0 | `NAV_VALID_ATTITUDE` | Attitude (quat/euler) is valid — always set when IMU is running |
| 1 | `NAV_VALID_POSITION` | `pos_ned` is valid — not yet implemented |
| 2 | `NAV_VALID_VELOCITY` | `vel_ned` is valid — not yet implemented |
| 3 | `NAV_VALID_BARO_ALT` | Baro altitude is valid — not yet implemented |

Guidance and Control must check `NAV_VALID_ATTITUDE` before using attitude data.

### Mission modes (`umsg_mission_mode_t`)

Defined in `msgs/umsg/defs/mission.json`:

```c
EMISSION_MODE_ALTITUDE_HOLD = 0   // the flight mode (implemented)
EMISSION_MODE_POSITION_HOLD = 1   // stub
EMISSION_MODE_WAYPOINT      = 2   // stub
EMISSION_MODE_RTL           = 3   // stub
```

There is **one** flight mode and `mission.c` selects it unconditionally - there is
no mode switch and no manual alternative. It holds altitude on the throttle stick
(which commands a climb rate) and attitude on roll and pitch. `EMISSION_MODE_MANUAL`
was removed with the second mode; the degraded-sensor paths that used to look like
it are still there, in `guidance.c`: throttle passthrough without
`NAV_VALID_BARO_ALT`, and a rate command without `NAV_VALID_ATTITUDE`. Those are
sensor failures, not modes.

---

## Adding or Changing a Message Type

1. Edit the appropriate JSON in `Firmware/msgs/umsg/defs/`
2. Regenerate, normally via the board tool (it does proto + umsg together):
   ```
   python Scripts/board.py gen
   ```
   The underlying call is:
   ```
   python Vendor/umsg/umsg_gen/umsg_gen/umsg_gen.py -d Firmware/msgs/umsg/defs -o Firmware/msgs/umsg
   ```
   Generated `umsg_<topic>.h` / `<topic>.c` land in `Firmware/msgs/umsg/`. The umsg
   *core* is not generated - it is compiled straight from `Vendor/umsg/umsg_lib/core/`
   (see `UMSG_ROOT` in the root `CMakeLists.txt`).
3. Re-run CMake configure
4. Update any layer that subscribes to or publishes the changed type

---

## What Is Not Yet Implemented

- **RC failsafe behaviour** — the receiver driver *detects* a lost link (`Rx_IsLinkUp()`, 1 s per the
  CRSF spec) and reports it through `link_quality` and a log line, but nothing acts on it: the
  channels keep their last values and the vehicle keeps flying them. Choosing an action is open, and
  a controlled descent is blocked on the missing altitude estimate below
- **Accel-aided horizontal estimation** — `pos_ned[0..1]` / `vel_ned[0..1]` come straight from the
  GPS fix at ~10 Hz and hold between fixes. Position hold wants them propagated through the
  accelerometer between fixes, the way the vertical channel already is. The vertical filter is the
  template; the missing piece is that the horizontal channels need the full body→NED rotation
  rather than just its bottom row
- **GPS altitude is not used at all** — the vertical estimate is baro-only. GPS altitude is
  decoded, published and echoed, but nothing blends it in, so there is no absolute MSL reference
  and no cross-check on a failed baro. Deciding how the two blend when they disagree by metres at
  different rates is still open
- **Autonomous guidance modes** — `guidance.c` implements the single altitude-hold/attitude mode only; `POSITION_HOLD`, `WAYPOINT` and `RTL` are enumerated but unwired
- **PID gain tuning** — gains come from `CFG_PID_*` macros in the target config; the rate loop is wired but untested

---

## Core Loop Notes

- `GetMicroseconds()` is available everywhere via `core/core_shared.h` for dt calculation
- `portMAX_DELAY` requires `#include "FreeRTOS.h"` (include path provided by CMake via `FREERTOS_ROOT/include`)
- Armed state for motor output comes from `control.c`'s cached `umsg_mission_state_t` subscription, not from the old `Fc_IsArmed()` / `g_Flight.isArmed`
- The old `fj_task.c` / `scheduler.c` loop is being replaced entirely — do not add new work there
