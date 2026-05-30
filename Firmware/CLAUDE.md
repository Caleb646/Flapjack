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

| Layer | Header | Source | Init fn | Update fn | Blocks on | Publishes |
|---|---|---|---|---|---|---|
| **Sensor: IMU** | `inc/sensors/imu.h` | `src/sensors/imu.c` | — | `SensorImu_Update()` | hardware (ISR/poll) | `umsg_sensors_imu_t` |
| **Sensor: Mag** | `inc/sensors/mag.h` | `src/sensors/mag.c` | — | `SensorMag_Update()` | hardware | `umsg_sensors_mag_t` |
| **Sensor: GPS** | `inc/sensors/gps.h` | `src/sensors/gps.c` | — | `SensorGps_Update()` | hardware | `umsg_sensors_gps_t` |
| **Sensor: Baro** | `inc/sensors/baro.h` | `src/sensors/baro.c` | — | `SensorBaro_Update()` | hardware | `umsg_sensors_baro_t` (stub) |
| **Sensor: RC** | `inc/sensors/rc.h` | `src/sensors/rc.c` | — | `SensorRc_Update()` | CM4 shared mem | `umsg_rc_input_t` |
| **Navigation** | `inc/nav/nav.h` | `src/nav/nav.c` | `Nav_Init()` | `Nav_Update()` | `umsg_sensors_imu_t` | `umsg_nav_state_t` |
| **Mission** | `inc/mission/mission.h` | `src/mission/mission.c` | `Mission_Init()` | `Mission_Update()` | `umsg_rc_input_t` | `umsg_mission_state_t` |
| **Guidance** | `inc/guidance/guidance.h` | `src/guidance/guidance.c` | `Guidance_Init()` | `Guidance_Update()` | `umsg_nav_state_t` | `umsg_guidance_setpoints_t` |
| **Control** | `inc/control/control.h` | `src/control/control.c` | `Control_Init()` | `Control_Update()` | `umsg_guidance_setpoints_t` | motors/servos (direct) |

Sensor wrappers have no `_Init` — they are called directly when new hardware data is available.

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

The natural task rates emerge from the blocking receive:
- **IMU task** → drives Nav at IMU ODR (400–800 Hz)
- **Nav task** → drives Guidance at IMU rate
- **Guidance task** → drives Control at nav rate
- **Mission task** → driven by RC link rate (~150 Hz)

Sensor tasks for Mag, GPS, Baro run at their own hardware rates independently.

---

## umsg Message Types

All message structs are in `Vendor/umsg/umsg_lib/inc/`. JSON definitions are the source of truth at `Firmware/msgs/umsg/`.

| Header | Struct | Key fields |
|---|---|---|
| `umsg_sensors.h` | `umsg_sensors_imu_t` | `gyro[3]` (deg/s), `accel[3]` (m/s²), `temperature` |
| `umsg_sensors.h` | `umsg_sensors_mag_t` | `field[3]` (normalised) |
| `umsg_sensors.h` | `umsg_sensors_baro_t` | `pressure`, `temperature` |
| `umsg_sensors.h` | `umsg_sensors_gps_t` | `lat`, `lon` (double, degrees), `alt`, `speed`, `course`, `fix_type`, `sats` |
| `umsg_rc.h` | `umsg_rc_input_t` | `channels[16]` (µs, 1000–2000), `rssi`, `link_quality` |
| `umsg_nav.h` | `umsg_nav_state_t` | `quat[4]`, `euler[3]` (deg), `gyro[3]` (deg/s), `pos_ned[3]`, `vel_ned[3]`, `alt`, `valid` |
| `umsg_mission.h` | `umsg_mission_state_t` | `mode` (`eMissionMode_t`), `target_pos[3]`, `target_heading`, `armed` |
| `umsg_guidance.h` | `umsg_guidance_setpoints_t` | `w[3]` (rad/s), `vel_b[3]` (body vel, `[2]`=throttle 0–1), `quat[4]` |

### Unit conventions

- **Gyro rates in `umsg_sensors_imu_t` and `umsg_nav_state_t`**: degrees/second — matches the IMU driver and Madgwick filter input.
- **Guidance `w[]`**: radians/second — `control.c` converts to deg/s via `RAD2DEG()` before the rate PID.

### NAV_VALID bits (`umsg_nav_state_t.valid`)

Defined in `inc/nav/nav.h`:

| Bit | Macro | Meaning |
|---|---|---|
| 0 | `NAV_VALID_ATTITUDE` | Attitude (quat/euler) is valid — always set when IMU is running |
| 1 | `NAV_VALID_POSITION` | `pos_ned` is valid — not yet implemented |
| 2 | `NAV_VALID_VELOCITY` | `vel_ned` is valid — not yet implemented |
| 3 | `NAV_VALID_BARO_ALT` | Baro altitude is valid — not yet implemented |

Guidance and Control must check `NAV_VALID_ATTITUDE` before using attitude data.

### Mission modes (`eMissionMode_t`)

Defined in `inc/mission/mission.h`:

```c
eMISSION_MODE_MANUAL        = 0   // RC sticks → rate setpoints (implemented)
eMISSION_MODE_ALTITUDE_HOLD = 1   // stub
eMISSION_MODE_POSITION_HOLD = 2   // stub
eMISSION_MODE_WAYPOINT      = 3   // stub
eMISSION_MODE_RTL           = 4   // stub
```

---

## Adding or Changing a Message Type

1. Edit the appropriate JSON in `Firmware/msgs/umsg/`
2. Regenerate the C library:
   ```
   python Vendor/umsg/umsg_gen/umsg_gen/umsg_gen.py -d Firmware/msgs/umsg -o Vendor/umsg/umsg_lib
   ```
3. Re-run CMake configure (CMakeLists.txt globs `umsg_lib/src/*.c`)
4. Update any layer that subscribes to or publishes the changed type

---

## What Is Not Yet Implemented

These are left for the FreeRTOS implementation agent:

- **FreeRTOS task creation** — no tasks exist yet; all layers are written but nothing calls them
- **RC umsg publisher** — `SensorRc_Update()` reads from `g_Rx` (updated by CM4) and publishes `umsg_rc_input_t`, but it has no task/caller yet; without it, Mission and Guidance block forever
- **Position / velocity estimation** — `nav.pos_ned`, `nav.vel_ned`, `nav.alt` are zeroed; `NAV_VALID_POSITION/VELOCITY/BARO_ALT` are never set
- **Autonomous guidance modes** — only `eMISSION_MODE_MANUAL` is implemented in `guidance.c`
- **PID gain tuning** — gains come from `CFG_PID_*` macros in the target config; the rate loop is wired but untested

---

## Core Loop Notes

- `GetMicroseconds()` is available everywhere via `core/core_shared.h` for dt calculation
- `portMAX_DELAY` requires `#include "FreeRTOS.h"` (include path provided by CMake via `FREERTOS_ROOT/include`)
- Armed state for motor output is checked from `umsg_mission_state_peek()` in `control.c`, not from the old `Fc_IsArmed()` / `g_Flight.isArmed`
- The old `fj_task.c` / `scheduler.c` loop is being replaced entirely — do not add new work there
