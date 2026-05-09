# Flapjack Firmware

STM32H745 dual-core (CM7 + CM4) flight controller firmware, written in C.

## Architecture

### Dual-core split
- **CM7** — flight-critical path: IMU, attitude estimation, PID, mixer, motor/servo output
- **CM4** — receiver path: CRSF UART decode, RX channel processing

Both cores run `Scheduler_Main()` from `Firmware/Common/Src/scheduler.c` with their own task table (`m_SequentialTasks[CM7_IDX]` / `m_SequentialTasks[CM4_IDX]`).

### Boot sequence
```
CM7: SPI → UART → Core_Init(timer+sync+logger) → SerialDebug → [wait CM4]
CM4:                Core_Init(timer+sync+logger) → [signal CM7 ready]
CM7:                                                             → IMU → Mag → PID → FC → RC → Scheduler
CM4: Rx_Init → Scheduler
```
`Core_Init` (`Firmware/Common/Src/core/core.c`) must complete on both cores before either enters its scheduler — it sets up the DWT debug timer (`CoreShared_Init`), inter-core mailboxes (`SyncInit`), and shared ring-buffer logger (`LoggerInit`).

### Inter-core communication
SEV-based mailbox in `Firmware/Common/Src/core/sync.c`:
- Sender: `memcpy` task → shared mailbox → `dsb` → `asm("sev")`
- Receiver: SEV IRQ → `SyncIRQHandler` → push to `SyncTaskQueue` (capacity 64)
- Main loop: `SyncProcessTasks()` dispatches via registered handlers

`TaskInterCoreSync` drains the queue. It is **async** (runs only when loop budget remains) — under sustained CM7 overrun, cross-core messages queue up and can be dropped silently at depth 64.

Currently the only cross-core task is `eSYNC_TASKID_UART_OUT`: CM7 writes a log line to its ring buffer, fires SEV, CM4 drains to UART.

### Logger
Two ring buffers in shared SRAM (`Firmware/Common/Src/core/log/logger.c`):
- CM7: 4096 bytes, CM4: 1024 bytes
- CM4 is `CFG_PRIMARY_LOGGER` — owns UART output
- CM7 flushes on `\n` via `SyncNotifyTaskUartOut`
- Both buffers are initialized once (HSEM-guarded idempotent init)

### Shared memory
`FJ_DEFINE_SHARED` / `FJ_DECLARE_SHARED` place data in a linker-defined shared SRAM section visible to both cores. Key shared globals: `g_Flight`, `g_Pid`, `g_Mixer`, `g_Rx`, `g_Servos`, `ga_MailBoxes`.

## CM7 Flight Loop (`Firmware/Common/Src/scheduler.c`)

Sequential tasks run every tick in order, deducting from the µs budget:
1. `Task_RcUpdate` — reads `g_Rx.channels[]`, maps to `Flight_t.target[]` (**stub — not yet implemented**)
2. `TaskImu_Update` — polls IMU via SPI → `IMU_t.accelData/gyroData`
3. `TaskAttitudeUpdate` — Madgwick filter → `Flight_t.current[ROLL/PITCH/YAW]`
4. `TaskPIDUpdate` — PID on `current` vs `target` → `Pid_t.data[]` (normalized [-1, 1])
5. `TaskMixerUpdate` — `Mixer_Mix` (compute) then `Mixer_Update` (write hardware, only if armed)

Async tasks run when budget remains, at their `hzUpdate` rate:
- `TaskInterCoreSync`, `Task_LogFlightData` (10Hz), `Task_LogHeartBeat` (10Hz)

## Key data structures

| Struct | File | Role |
|---|---|---|
| `Flight_t` | `Common/Inc/flight.h` | Central data bus: `current[]`, `target[]`, `max[]`, `attitudeFilter` |
| `Pid_t` | `Common/Inc/mc/pid.h` | Per-axis PID state and normalized output `data[]` |
| `Mixer_t` | `Common/Inc/mc/mixer.h` | Profile table, `motorOutputs[]`, `servoOutputs[]` |
| `Rx_t` | `Common/Inc/drivers/rx/rx.h` | 16 CRSF channels in RC units |
| `IMU_t` | `Common/Inc/device/imu/imu.h` | Accel/gyro data, calibration state |

## Mixer profiles (`Firmware/Common/Src/mc/mixer.c`)

Three profiles defined; **hardcoded to `TILT_ROTOR`** at init (TODO: read from config):
- `TEST_SETUP` — 1 motor, 1 servo
- `TILT_ROTOR` — 2 motors (throttle only), 2 servos (all axes)
- `AIRPLANE` — 2 motors, 4 servos (ailerons, elevator, rudder)

Servo outputs are in µs [500, 2500], initialized to center (1500) each tick, with each mix entry adding `pidData[axis] * 1000` µs as a signed delta.

## Known gaps (not yet implemented)

- **`Rc_Update` is a stub** (`Firmware/Common/Src/fc/rc.c`): reads `g_Rx.channels[]` but never writes to `Flight_t.target[]` or implements arm/disarm logic. The vehicle cannot be commanded.
- **Mixer profile** is hardcoded to `TILT_ROTOR`; config-file selection not implemented.
- **`g_Rx.channels[]`** is written by CM4 and read by CM7 with no cross-core lock — frame consistency across 16 channels is not guaranteed.

## Key patterns

- **Error propagation**: functions return `eSTATUS_t` (`eSTATUS_SUCCESS` / `eSTATUS_FAILURE`). Use `STATUS_FAIL()` macro to check. `RETURN_IF(cond, status, msg)` logs and early-returns.
- **Inline wrappers**: public API functions in headers (e.g. `Fc_Init`, `Pid_Update`, `Mag_Init`) are `static inline` wrappers over `_`-suffixed internal functions that take explicit struct pointers. Pass the global singleton implicitly.
- **Task signature**: `eSTATUS_t taskFn(uint32_t usCurrentTime, uint32_t usDeltaTime)` — `usDeltaTime` is microseconds since last execution of this task.
- **Servo µs**: `SERVO_LEFT_US_DC=500`, `SERVO_CENTER_US_DC=1500`, `SERVO_RIGHT_US_DC=2500`. Timer prescaler is 64-1 at 64MHz → 1µs per tick.

## Directory structure

```
Firmware/
  Common/
    Inc/          — headers
      core/       — core_shared.h, sync.h, log/
      device/     — imu/, serial/
      drivers/    — bus/ (spi, uart, dma), rx/ (crsf), sensors/ (mag), mc/ (dshot, motors, servos)
      fc/         — rc.h
      mc/         — filter.h, mixer.h, pid.h
      mem/        — queue.h, ring_buff.h, umap.h, vector.h
    Src/          — implementations mirroring Inc/
  Targets/        — target.h per board (pin assignments, clock config, PID gains)
  HwTests/        — hardware-in-loop test harness
Tests/            — unit tests (CMake, stubs in Tests/stubs/)
```

## Graphify knowledge graph

Two separate knowledge graphs cover this repo:

| Graph | Covers | Report |
|---|---|---|
| `graphify-out/graph.json` | Firmware + Scripts | `graphify-out/GRAPH_REPORT.md` |
| `graphify-out/tests/graph.json` | Tests (unit tests + stubs) | `graphify-out/tests/GRAPH_REPORT.md` |

Open `graphify-out/graph.html` in a browser for an interactive view of the main graph.

Query the main graph: `/graphify query "<question>"`
Query the tests graph: `/graphify query "<question>" --graph graphify-out/tests/graph.json`
